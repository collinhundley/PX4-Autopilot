// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#include "battery.h"
#include <gtest/gtest.h>

class BatteryTest : public ::testing::Test
{
protected:
	hrt_abstime now{1000000};

	void set(const char *name, float value)
	{
		ASSERT_EQ(param_set_no_notification(param_find(name), &value), 0) << name;
	}

	void SetUp() override
	{
		param_control_autosave(false); // Functional tests do not start the autosave work queue.
		param_reset_all();
		const int32_t cells = 4;
		ASSERT_EQ(param_set_no_notification(param_find("BAT1_N_CELLS"), &cells), 0);
		set("BAT1_V_EMPTY", 3.5f);
		set("BAT1_V_CHARGED", 4.2f);
		set("BAT1_R_INTERNAL", 0.02f);
		set("BAT1_CAPACITY", 0.f);
		set("BAT1_I_OVERWRITE", 0.f);
	}

	void step(Battery &battery, float voltage, float current, unsigned count = 1)
	{
		for (unsigned i = 0; i < count; ++i) {
			battery.updateVoltage(voltage);
			battery.updateCurrent(current);
			battery.updateBatteryStatus(now);
			now += 50000;
		}
	}
};

TEST_F(BatteryTest, CompensatedVoltageUsesSocFilterAndConfiguredResistance)
{
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.6f, 10.f, 60);
	const auto status = battery.getBatteryStatus();
	EXPECT_FLOAT_EQ(status.voltage_v, 14.6f);
	EXPECT_NEAR(status.voltage_v_compensated, 15.4f, 0.0001f);
	EXPECT_NEAR(status.remaining, 0.5f, 0.0001f);
	// The diagnostic OCV estimate uses estimated resistance, not our override.
	EXPECT_GT(fabsf(status.voltage_v_compensated - status.ocv_estimate_filtered), 0.1f);

	step(battery, 14.6f, 20.f);
	const auto filtered = battery.getBatteryStatus();
	EXPECT_GT(filtered.voltage_v_compensated, status.voltage_v_compensated);
	EXPECT_LT(filtered.voltage_v_compensated, 16.2f); // No unfiltered jump to V + I*R.
	EXPECT_NEAR(filtered.remaining, (filtered.voltage_v_compensated / 4.f - 3.5f) / 0.7f, 0.0001f);
}

TEST_F(BatteryTest, CompensatedVoltageUsesEstimatedResistanceWhenSelected)
{
	set("BAT1_R_INTERNAL", -1.f);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.6f, 10.f, 60);
	const auto status = battery.getBatteryStatus();
	EXPECT_GT(status.voltage_v_compensated, status.voltage_v);
	EXPECT_NEAR(status.voltage_v_compensated, status.ocv_estimate_filtered, 0.0001f);
	EXPECT_NEAR(status.remaining, (status.voltage_v_compensated / 4.f - 3.5f) / 0.7f, 0.0001f);
}

TEST_F(BatteryTest, MissingBatteryInitializationOrCellCountDoesNotPublishAnEstimate)
{
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().voltage_v_compensated, 0.f);
	battery.setConnected(true);
	step(battery, 14.6f, 10.f);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().voltage_v_compensated, 0.f);
	step(battery, 14.6f, 10.f, 60);
	EXPECT_GT(battery.getBatteryStatus().voltage_v_compensated, 0.f);
	battery.setConnected(false);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().voltage_v_compensated, 0.f);

	const int32_t cells = 0;
	ASSERT_EQ(param_set_no_notification(param_find("BAT1_N_CELLS"), &cells), 0);
	Battery unknown_cells(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	unknown_cells.setConnected(true);
	step(unknown_cells, 14.6f, 10.f, 60);
	EXPECT_FLOAT_EQ(unknown_cells.getBatteryStatus().voltage_v_compensated, 0.f);
}

TEST_F(BatteryTest, PartiallyChargedBatteryStartsFromVoltageEvenWithKnownCapacity)
{
	set("BAT1_R_INTERNAL", 0.f);
	set("BAT1_CAPACITY", 4000.f);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 15.4f, 0.f, 60);
	const auto status = battery.getBatteryStatus();
	EXPECT_FLOAT_EQ(status.discharged_mah, 0.f);
	EXPECT_NEAR(status.remaining, 0.5f, 0.0001f);
	step(battery, 15.4f, 8.f, 20);
	const auto discharged = battery.getBatteryStatus();
	EXPECT_GT(discharged.discharged_mah, 0.f);
	EXPECT_LT(discharged.remaining, status.remaining);
	EXPECT_LT(discharged.remaining, 1.f - discharged.discharged_mah / 4000.f);
}

TEST_F(BatteryTest, NoLoadVoltageAndWarningsContinueUsingTheSameSocFilter)
{
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);
	step(battery, 14.6f, 10.f, 60);
	const float before = battery.getBatteryStatus().voltage_v_compensated;
	step(battery, 14.6f, 0.f);
	EXPECT_LT(battery.getBatteryStatus().voltage_v_compensated, before);
	step(battery, 13.f, 0.f, 200);
	const auto empty = battery.getBatteryStatus();
	EXPECT_LT(empty.voltage_v_compensated, 14.f);
	EXPECT_FLOAT_EQ(empty.remaining, 0.f);
	EXPECT_EQ(empty.warning, battery_status_s::WARNING_EMERGENCY);
}
