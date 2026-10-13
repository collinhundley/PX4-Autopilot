// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#include "batteryCheck.hpp"
#include <lib/battery/battery.h>
#include <gtest/gtest.h>
#include <uORB/PublicationMulti.hpp>

/* EVENT
 * @skip-file
 */

class BatteryChecksTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
		set("COM_ARM_BAT_MIN", -1.f); // SOC availability must not depend on the minimum-charge threshold.
		battery_status_s disconnected{};
		_primary_pub.publish(disconnected);
		_secondary_pub.publish(disconnected);
	}

	void TearDown() override
	{
		battery_status_s disconnected{};
		_primary_pub.publish(disconnected);
		_secondary_pub.publish(disconnected);
	}

	void set(const char *name, float value)
	{
		ASSERT_EQ(param_set_no_notification(param_find(name), &value), 0) << name;
	}

	void profile(bool configured)
	{
		const int32_t mode = battery_soc_status_s::MODE_ACTIVE;
		const int32_t cells = 4;
		ASSERT_EQ(param_set_no_notification(param_find("BAT1_SOC_MODE"), &mode), 0);
		ASSERT_EQ(param_set_no_notification(param_find("BAT1_N_CELLS"), &cells), 0);
		set("BAT1_CAPACITY", 4000.f);
		set("BAT1_R_INTERNAL", 0.f);
		set("BAT_SOC_SETTLE", 120.f);

		if (configured) {
			for (unsigned i = 0; i < BatterySocEstimator::PROFILE_POINTS; ++i) {
				char name[17];
				snprintf(name, sizeof(name), "BAT1_SOC_V%u", i);
				const float soc = i / static_cast<float>(BatterySocEstimator::PROFILE_POINTS - 1);
				set(name, 3.3f + 0.9f * soc * soc); // Synthetic profile, not a LiPo calibration.
			}
		}
	}

	battery_status_s measurement(float soc)
	{
		battery_status_s battery{};
		battery.timestamp = hrt_absolute_time();
		battery.connected = true;
		battery.voltage_v = 15.f;
		battery.current_a = 5.f;
		battery.remaining = soc;
		battery.time_remaining_s = NAN;
		return battery;
	}

	bool canArm(bool armed = false)
	{
		BatteryChecks check;
		vehicle_status_s status{};
		status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
		Context context{status};
		failsafe_flags_s flags{};
		Report reporter{flags, 0_s};
		check.checkAndReport(context, reporter);
		return reporter.armingCheckResults().can_arm == NavModes::All
		       && reporter.armingCheckResults().can_run == NavModes::All;
	}

	uORB::PublicationMulti<battery_status_s> _primary_pub{ORB_ID(battery_status)};
	uORB::PublicationMulti<battery_status_s> _secondary_pub{ORB_ID(battery_status)};
};

TEST_F(BatteryChecksTest, RejectsUnavailableSocEvenWithMinimumChargeCheckDisabled)
{
	for (float unavailable : {-1.f, NAN, INFINITY, 1.1f}) {
		_primary_pub.publish(measurement(unavailable));
		EXPECT_FALSE(canArm());
	}
}

TEST_F(BatteryChecksTest, DefinedSocDoesNotRequireRelaxationToArm)
{
	for (float soc : {0.f, 0.5f, 1.f}) {
		_primary_pub.publish(measurement(soc));
		EXPECT_TRUE(canArm()); // The source's SOC warning and configured minimum still govern low-charge arming.
	}
}

TEST_F(BatteryChecksTest, AnUnavailableSecondaryBatteryAlsoPreventsArming)
{
	_primary_pub.publish(measurement(0.8f));
	_secondary_pub.publish(measurement(NAN));
	EXPECT_FALSE(canArm());
	_secondary_pub.publish(measurement(0.6f));
	EXPECT_TRUE(canArm());
}

TEST_F(BatteryChecksTest, ActiveVoltageStartupBlocksArmingUntilReadyThenAllowsItWithoutRest)
{
	profile(true);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);

	for (unsigned i = 0; i < 43; ++i) {
		battery.updateVoltage(14.1f);
		battery.updateCurrent(5.f); // Above the correction current limit throughout startup.
		battery.updateBatteryStatus(1000000 + i * 50000);
		_primary_pub.publish(battery.getBatteryStatus());

		if (i == 0) {
			EXPECT_FALSE(canArm());
		}
	}

	EXPECT_TRUE(battery.getSocEstimatorStatus().initialized);
	EXPECT_TRUE(battery.getSocEstimatorStatus().active);
	EXPECT_FALSE(battery.getSocEstimatorStatus().correction_eligible);
	EXPECT_NEAR(battery.getBatteryStatus().remaining, 0.5f, 0.0001f);
	EXPECT_TRUE(canArm());
}

TEST_F(BatteryChecksTest, InvalidActiveProfileCannotFlyUsingTheKnownLegacyEstimate)
{
	profile(false);
	Battery battery(1, nullptr, 50000, battery_status_s::SOURCE_POWER_MODULE);
	battery.setConnected(true);

	for (unsigned i = 0; i < 60; ++i) {
		battery.updateVoltage(16.8f);
		battery.updateCurrent(0.f);
		battery.updateBatteryStatus(1000000 + i * 50000);
	}

	ASSERT_GT(battery.getSocEstimatorStatus().legacy_soc, 0.9f);
	EXPECT_FALSE(battery.getSocEstimatorStatus().initialized);
	EXPECT_FLOAT_EQ(battery.getBatteryStatus().remaining, -1.f);
	_primary_pub.publish(battery.getBatteryStatus());
	EXPECT_FALSE(canArm());
}

TEST_F(BatteryChecksTest, UnavailableSocDoesNotPreventRunningAnAlreadyArmedVehicle)
{
	_primary_pub.publish(measurement(NAN));
	EXPECT_TRUE(canArm(true)); // Missing startup SOC adds an arming check, not a forced in-flight mode change.
}
