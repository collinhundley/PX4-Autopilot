/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include <gtest/gtest.h>

// Registration already happens in translation_node_lib; test the converters.
#define REGISTER_TOPIC_TRANSLATION_DIRECT(class_name)
#include "translations/translation_battery_status_v1.h"
#include "translations/translation_battery_status_v2.h"
#undef REGISTER_TOPIC_TRANSLATION_DIRECT

TEST(BatteryStatusTranslation, OlderMessagesPreserveChargeWithoutInventingCompensatedVoltage)
{
	px4_msgs_old::msg::BatteryStatusV0 old;
	old.timestamp = 123456;
	old.connected = true;
	old.voltage_v = 15.f;
	old.remaining = 0.37f;
	old.discharged_mah = 0.f;
	old.cell_count = 4;
	old.voltage_cell_v[0] = 3.75f;
	old.ocv_estimate_filtered = 16.f;
	px4_msgs_old::msg::BatteryStatusV1 intermediate;
	BatteryStatusV1Translation::fromOlder(old, intermediate);
	px4_msgs::msg::BatteryStatus current;
	current.voltage_v_compensated = 99.f; // A reused destination must be cleared.
	BatteryStatusV2Translation::fromOlder(intermediate, current);
	EXPECT_EQ(current.timestamp, old.timestamp);
	EXPECT_TRUE(current.connected);
	EXPECT_FLOAT_EQ(current.remaining, 0.37f);
	EXPECT_FLOAT_EQ(current.discharged_mah, 0.f);
	EXPECT_FLOAT_EQ(current.voltage_v, 15.f);
	EXPECT_EQ(current.voltage_cell_v, intermediate.voltage_cell_v);
	EXPECT_FLOAT_EQ(current.ocv_estimate_filtered, 16.f);
	EXPECT_FLOAT_EQ(current.voltage_v_compensated, 0.f);
}

TEST(BatteryStatusTranslation, DowngradePreservesMeasuredVoltageAndCharge)
{
	px4_msgs::msg::BatteryStatus current;
	current.voltage_v = 15.f;
	current.voltage_v_compensated = 15.8f;
	current.remaining = 0.37f;
	current.warning = 2;
	current.voltage_cell_v[13] = 3.75f;
	current.estimation_covariance_norm = 1.25f;
	px4_msgs_old::msg::BatteryStatusV1 intermediate;
	BatteryStatusV2Translation::toOlder(current, intermediate);
	EXPECT_FLOAT_EQ(intermediate.voltage_v, 15.f);
	EXPECT_FLOAT_EQ(intermediate.remaining, 0.37f);
	EXPECT_EQ(intermediate.warning, 2);
	EXPECT_EQ(intermediate.voltage_cell_v, current.voltage_cell_v);
	EXPECT_FLOAT_EQ(intermediate.estimation_covariance_norm, 1.25f);
	px4_msgs_old::msg::BatteryStatusV0 old;
	BatteryStatusV1Translation::toOlder(intermediate, old);
	EXPECT_FLOAT_EQ(old.remaining, 0.37f);
	EXPECT_FLOAT_EQ(old.voltage_v, 15.f);
	EXPECT_EQ(old.warning, 2);
	EXPECT_EQ(old.voltage_cell_v, current.voltage_cell_v);
}
