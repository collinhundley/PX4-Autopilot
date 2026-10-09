// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#include "OsdTelemetry.hpp"
#include <gtest/gtest.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <matrix/matrix/math.hpp>
#include <cstring>

using msp_osd::OsdTelemetryCore;

class OsdTelemetryTest : public ::testing::Test
{
protected:
	static constexpr uint64_t SECOND = 1000000;
	uint64_t now{10 * SECOND};
	OsdTelemetryCore core;
	OsdTelemetryCore::Samples samples;
	OsdTelemetryCore::Settings settings;

	void refresh()
	{
		samples.battery.timestamp = now;
		samples.airspeed.timestamp = now;
		samples.position.timestamp = now;
		samples.global.timestamp = now;
		samples.attitude.timestamp = now;
		samples.status.timestamp = now;
		samples.land.timestamp = now;
		samples.gnss.timestamp = now;
		samples.rc.timestamp_last_signal = now;
	}

	void SetUp() override
	{
		refresh();
		samples.battery.connected = true;
		samples.battery.voltage_v = 24.f;
		samples.battery.cell_count = 6;
		samples.battery.current_a = 12.f;
		samples.battery.voltage_v_compensated = 25.2f;
		samples.battery.remaining = 0.42f;
		samples.battery.discharged_mah = 400.f;
		samples.airspeed.airspeed_source = airspeed_validated_s::SOURCE_SENSOR_1;
		samples.airspeed.indicated_airspeed_m_s = 15.f;
		samples.airspeed.calibrated_airspeed_m_s = 17.f;
		samples.position.v_xy_valid = true;
		samples.position.v_z_valid = true;
		samples.position.vx = 3.f;
		samples.position.vy = 4.f;
		samples.position.vz = -2.f;
		samples.position.z_valid = true;
		samples.position.z = -110.f;
		samples.home.timestamp = 1;
		samples.home.valid_alt = true;
		samples.home.valid_lpos = true;
		samples.home.z = -10.f;
		samples.attitude.q[0] = 1.f;
		samples.status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
		samples.status.nav_state_display = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		samples.land.landed = true;
		samples.gnss.fix_type = sensor_gps_s::FIX_TYPE_3D;
		samples.gnss.latitude_deg = 33.0;
		samples.gnss.longitude_deg = -112.0;
		samples.gnss.satellites_used = 14;
		samples.rc.rssi = 85;
	}

	void update() { core.update(now, samples, settings); }

	void message(const char *text, uint8_t severity = 6, uint64_t timestamp = 0)
	{
		mavlink_log_s log{};
		log.timestamp = timestamp ? timestamp : now;
		log.severity = severity;
		strncpy(log.text, text, sizeof(log.text) - 1);
		core.pushMessage(now, log, settings);
	}
};

TEST_F(OsdTelemetryTest, ExtractsIndependentPhysicalQuantities)
{
	update();
	const auto &data = core.data();
	EXPECT_FLOAT_EQ(data.battery_voltage, 24.f);
	EXPECT_FLOAT_EQ(data.cell_voltage, 4.f);
	EXPECT_FLOAT_EQ(data.compensated_battery_voltage, 25.2f);
	EXPECT_FLOAT_EQ(data.compensated_cell_voltage, 4.2f);
	EXPECT_FLOAT_EQ(data.battery_remaining_percent, 42.f);
	EXPECT_FLOAT_EQ(data.current_a, 12.f);
	EXPECT_FLOAT_EQ(data.discharged_mah, 400.f);
	EXPECT_FLOAT_EQ(data.airspeed_m_s, 17.f);
	EXPECT_FALSE(data.airspeed_estimated);
	EXPECT_FLOAT_EQ(data.ground_speed_m_s, 5.f);
	EXPECT_FLOAT_EQ(data.altitude_m, 100.f);
	EXPECT_FLOAT_EQ(data.vertical_speed_m_s, 2.f);
	EXPECT_DOUBLE_EQ(data.latitude_deg, 33.0);
	EXPECT_DOUBLE_EQ(data.longitude_deg, -112.0);
	EXPECT_EQ(data.satellites, 14);
	EXPECT_FLOAT_EQ(data.rssi_percent, 85.f);
	EXPECT_FLOAT_EQ(data.throttle_percent, 0.f);
}

TEST_F(OsdTelemetryTest, ClearsStaleSensorValuesRatherThanRetainingLastValue)
{
	update();
	now += 4 * SECOND;
	update();
	const auto &data = core.data();
	EXPECT_TRUE(std::isnan(data.battery_voltage));
	EXPECT_TRUE(std::isnan(data.compensated_battery_voltage));
	EXPECT_TRUE(std::isnan(data.compensated_cell_voltage));
	EXPECT_TRUE(std::isnan(data.battery_remaining_percent));
	EXPECT_TRUE(std::isnan(data.current_a));
	EXPECT_TRUE(std::isnan(data.airspeed_m_s));
	EXPECT_TRUE(std::isnan(data.ground_speed_m_s));
	EXPECT_TRUE(std::isnan(data.altitude_m));
	EXPECT_TRUE(std::isnan(data.roll_rad));
	EXPECT_TRUE(std::isnan(data.latitude_deg));
	EXPECT_TRUE(std::isnan(data.rssi_percent));
	EXPECT_TRUE(std::isnan(data.throttle_percent));
	EXPECT_EQ(data.satellites, -1);
	EXPECT_FALSE(data.status_valid);
	EXPECT_FALSE(data.flight_time_valid);
}

TEST_F(OsdTelemetryTest, RejectsMissingFutureAndInvalidBatterySamples)
{
	samples.battery.timestamp = 0;
	update();
	EXPECT_TRUE(std::isnan(core.data().battery_voltage));
	samples.battery.timestamp = now + 1;
	update();
	EXPECT_TRUE(std::isnan(core.data().battery_voltage));
	samples.battery.timestamp = now;
	samples.battery.connected = false;
	update();
	EXPECT_TRUE(std::isnan(core.data().current_a));
	samples.battery.connected = true;
	samples.battery.current_a = -1.f;
	samples.battery.discharged_mah = -1.f;
	samples.battery.cell_count = 0;
	update();
	EXPECT_FLOAT_EQ(core.data().battery_voltage, 24.f);
	EXPECT_TRUE(std::isnan(core.data().cell_voltage));
	EXPECT_TRUE(std::isnan(core.data().current_a));
	EXPECT_TRUE(std::isnan(core.data().discharged_mah));
	samples.battery.current_a = INFINITY;
	samples.battery.discharged_mah = NAN;
	update();
	EXPECT_TRUE(std::isnan(core.data().current_a));
	EXPECT_TRUE(std::isnan(core.data().discharged_mah));
}

TEST_F(OsdTelemetryTest, CompensatedVoltageUsesPublishedSocVoltageWithoutDiagnosticFallback)
{
	samples.battery.ocv_estimate_filtered = 30.f;
	update();
	EXPECT_FLOAT_EQ(core.data().compensated_battery_voltage, 25.2f);
	EXPECT_FLOAT_EQ(core.data().compensated_cell_voltage, 4.2f);
	samples.battery.cell_count = 0;
	update();
	EXPECT_FLOAT_EQ(core.data().compensated_battery_voltage, 25.2f);
	EXPECT_TRUE(std::isnan(core.data().compensated_cell_voltage));

	for (float value : {0.f, -1.f, NAN, INFINITY}) {
		samples.battery.voltage_v_compensated = value;
		update();
		EXPECT_TRUE(std::isnan(core.data().compensated_battery_voltage));
		EXPECT_TRUE(std::isnan(core.data().compensated_cell_voltage));
		EXPECT_FLOAT_EQ(core.data().battery_voltage, 24.f);
	}
}

TEST_F(OsdTelemetryTest, RemainingUsesReportedFractionIncludingPartialChargeAndEmpty)
{
	samples.battery.capacity = 5000;
	samples.battery.discharged_mah = 0.f;
	samples.battery.remaining = 0.37f;
	update();
	EXPECT_FLOAT_EQ(core.data().battery_remaining_percent, 37.f);

	for (float value : {0.f, 1.f}) {
		samples.battery.remaining = value;
		update();
		EXPECT_FLOAT_EQ(core.data().battery_remaining_percent, value * 100.f);
	}

	for (float value : {-1.f, 1.01f, NAN, INFINITY}) {
		samples.battery.remaining = value;
		update();
		EXPECT_TRUE(std::isnan(core.data().battery_remaining_percent));
	}

	samples.battery.remaining = 0.37f;
	samples.battery.connected = false;
	update();
	EXPECT_TRUE(std::isnan(core.data().battery_remaining_percent));
	EXPECT_TRUE(std::isnan(core.data().compensated_battery_voltage));
	samples.battery.connected = true;
	samples.battery.timestamp = now + 1;
	update();
	EXPECT_TRUE(std::isnan(core.data().battery_remaining_percent));
	EXPECT_TRUE(std::isnan(core.data().compensated_battery_voltage));
}

TEST_F(OsdTelemetryTest, EstimatedAirspeedIsExplicitAndDisabledDoesNotBecomeGroundSpeed)
{
	samples.airspeed.airspeed_source = airspeed_validated_s::SOURCE_GROUND_MINUS_WIND;
	update();
	EXPECT_TRUE(core.data().airspeed_estimated);
	EXPECT_FLOAT_EQ(core.data().airspeed_m_s, 17.f);
	samples.airspeed.airspeed_source = airspeed_validated_s::SOURCE_SYNTHETIC;
	update();
	EXPECT_TRUE(core.data().airspeed_estimated);
	samples.airspeed.airspeed_source = airspeed_validated_s::SOURCE_DISABLED;
	update();
	EXPECT_TRUE(std::isnan(core.data().airspeed_m_s));
	EXPECT_FLOAT_EQ(core.data().ground_speed_m_s, 5.f);
	samples.airspeed.airspeed_source = airspeed_validated_s::SOURCE_SENSOR_2;
	samples.airspeed.calibrated_airspeed_m_s = NAN;
	update();
	EXPECT_TRUE(std::isnan(core.data().airspeed_m_s));
}

TEST_F(OsdTelemetryTest, CalibratedAirspeedDoesNotDependOnIndicatedAirspeed)
{
	// CAS is already calibrated by the airspeed selector; IAS validity does not
	// determine whether the independently published CAS can be displayed.
	samples.airspeed.indicated_airspeed_m_s = NAN;
	update();
	EXPECT_FLOAT_EQ(core.data().airspeed_m_s, 17.f);

	samples.airspeed.indicated_airspeed_m_s = 15.f;
	samples.airspeed.calibrated_airspeed_m_s = 0.f;
	update();
	EXPECT_FLOAT_EQ(core.data().airspeed_m_s, 0.f);

	for (float value : {-1.f, NAN, INFINITY, -INFINITY}) {
		samples.airspeed.calibrated_airspeed_m_s = value;
		update();
		EXPECT_TRUE(std::isnan(core.data().airspeed_m_s)); // No fallback to valid IAS.
	}

	samples.airspeed.calibrated_airspeed_m_s = 19.f;
	update();
	EXPECT_FLOAT_EQ(core.data().airspeed_m_s, 19.f);
}

TEST_F(OsdTelemetryTest, AltitudeNeverFallsBackToDifferentReference)
{
	samples.home.valid_lpos = false;
	samples.gnss.altitude_msl_m = 5000.0;
	update();
	EXPECT_TRUE(std::isnan(core.data().altitude_m));
	samples.home.valid_lpos = true;
	update();
	EXPECT_FLOAT_EQ(core.data().altitude_m, 100.f);
	samples.position.z_valid = false;
	update();
	EXPECT_TRUE(std::isnan(core.data().altitude_m));
}

TEST_F(OsdTelemetryTest, LocalHomeAltitudeDoesNotRequireGlobalReference)
{
	samples.home.valid_alt = false;
	samples.home.valid_hpos = false;
	samples.home.alt = NAN;
	samples.home.lat = static_cast<double>(NAN);
	samples.home.lon = static_cast<double>(NAN);
	samples.gnss.fix_type = sensor_gps_s::FIX_TYPE_NONE;
	update();
	EXPECT_FLOAT_EQ(core.data().altitude_m, 100.f);
	EXPECT_TRUE(std::isnan(core.data().home_distance_m));
	samples.home.z = NAN;
	update();
	EXPECT_TRUE(std::isnan(core.data().altitude_m));
	samples.home.z = -10.f;
	samples.home.timestamp = now + 1;
	update();
	EXPECT_TRUE(std::isnan(core.data().altitude_m));
}

TEST_F(OsdTelemetryTest, ChecksVelocityFlagsAndRcLinkValidity)
{
	samples.position.v_xy_valid = false;
	samples.position.v_z_valid = false;
	samples.rc.rc_lost = true;
	update();
	EXPECT_TRUE(std::isnan(core.data().ground_speed_m_s));
	EXPECT_TRUE(std::isnan(core.data().vertical_speed_m_s));
	EXPECT_TRUE(std::isnan(core.data().rssi_percent));
	samples.rc.rc_lost = false;
	samples.rc.rssi = -1;
	update();
	EXPECT_TRUE(std::isnan(core.data().rssi_percent));
	samples.rc.rssi = 0;
	update();
	EXPECT_FLOAT_EQ(core.data().rssi_percent, 0.f);
	samples.rc.rc_failsafe = true;
	update();
	EXPECT_TRUE(std::isnan(core.data().rssi_percent));
}

TEST_F(OsdTelemetryTest, HomeDistanceDoesNotOverflowAtThirtyTwoKilometres)
{
	samples.global.lat_lon_valid = true;
	samples.global.lat = 0.0;
	samples.global.lon = 0.0;
	samples.home.valid_hpos = true;
	samples.home.lat = 1.0;
	samples.home.lon = 0.0;
	update();
	EXPECT_GT(core.data().home_distance_m, 100000.f);
	EXPECT_LT(core.data().home_distance_m, 120000.f);
	EXPECT_NEAR(core.data().home_bearing_rad, 0.f, 1e-5f);
	samples.home.lat = 0.0;
	samples.home.lon = 1.0;
	update();
	EXPECT_NEAR(core.data().home_bearing_rad, 1.57079632679f, 1e-5f);
	samples.home.lon = 0.0;
	update();
	EXPECT_FLOAT_EQ(core.data().home_distance_m, 0.f);
	EXPECT_TRUE(std::isnan(core.data().home_bearing_rad));
	samples.global.lat_lon_valid = false;
	update();
	EXPECT_TRUE(std::isnan(core.data().home_distance_m));
}

TEST_F(OsdTelemetryTest, RejectsImpossibleCoordinatesAndGnssWithoutFix)
{
	samples.global.lat_lon_valid = true;
	samples.global.lat = 91.0;
	samples.home.valid_hpos = true;
	samples.gnss.fix_type = sensor_gps_s::FIX_TYPE_NONE;
	update();
	EXPECT_TRUE(std::isnan(core.data().home_distance_m));
	EXPECT_TRUE(std::isnan(core.data().latitude_deg));
	EXPECT_EQ(core.data().satellites, 14);
}

TEST_F(OsdTelemetryTest, AttitudePreservesBodyQuaternionAndRejectsCorruption)
{
	const matrix::Quatf attitude(matrix::Eulerf(0.2f, 0.3f, 0.4f));
	attitude.copyTo(samples.attitude.q);
	update();
	EXPECT_NEAR(core.data().roll_rad, 0.2f, 1e-5f);
	EXPECT_NEAR(core.data().pitch_rad, 0.3f, 1e-5f);

	for (unsigned i = 0; i < 4; ++i) {
		EXPECT_NEAR(core.data().attitude_q[i], attitude(i), 1e-5f);
		samples.attitude.q[i] = 0.f;
	}

	update();
	EXPECT_TRUE(std::isnan(core.data().roll_rad));
	EXPECT_TRUE(std::isnan(core.data().attitude_q[0]));
	samples.attitude.q[0] = NAN;
	update();
	EXPECT_TRUE(std::isnan(core.data().pitch_rad));
}

TEST_F(OsdTelemetryTest, ModeUsesDisplayStateAndShowsVtolTransitions)
{
	samples.status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	samples.status.is_vtol = true;
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	update();
	EXPECT_STREQ(core.data().mode, "Mission FW");
	samples.status.in_transition_mode = true;
	samples.status.in_transition_to_fw = false;
	update();
	EXPECT_STREQ(core.data().mode, "Mission >MC");
	samples.status.nav_state_display = UINT8_MAX;
	update();
	EXPECT_STREQ(core.data().mode, "Unknown >MC");
}

TEST_F(OsdTelemetryTest, PositionModeUsesCruiseForFixedWingIncludingVtol)
{
	samples.status.nav_state = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	samples.status.nav_state_display = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	update();
	EXPECT_STREQ(core.data().mode, "Position");

	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	update();
	EXPECT_STREQ(core.data().mode, "Cruise");

	samples.status.is_vtol = true;
	update();
	EXPECT_STREQ(core.data().mode, "Cruise FW");

	samples.status.in_transition_mode = true;
	samples.status.in_transition_to_fw = false;
	update();
	EXPECT_STREQ(core.data().mode, "Cruise >MC");

	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	samples.status.in_transition_to_fw = true;
	update();
	EXPECT_STREQ(core.data().mode, "Position >FW");

	samples.status.in_transition_mode = false;
	update();
	EXPECT_STREQ(core.data().mode, "Position MC");
}

TEST_F(OsdTelemetryTest, TimerStartsAtTakeoffFreezesOnLandingAndResetsOnNextFlight)
{
	update();
	EXPECT_EQ(core.data().flight_seconds, 0u);
	samples.status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	update();
	EXPECT_EQ(core.data().flight_seconds, 0u);
	samples.status.takeoff_time = now;
	samples.land.landed = false;
	update();
	now += 65 * SECOND;
	refresh();
	update();
	EXPECT_EQ(core.data().flight_seconds, 65u);
	samples.land.landed = true;
	update();
	now += 10 * SECOND;
	refresh();
	update();
	EXPECT_EQ(core.data().flight_seconds, 65u);
	samples.status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	samples.status.takeoff_time = 0;
	update();
	EXPECT_EQ(core.data().flight_seconds, 65u);
	now += SECOND;
	refresh();
	samples.status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	samples.status.takeoff_time = now;
	samples.land.landed = false;
	update();
	EXPECT_EQ(core.data().flight_seconds, 0u);
}

TEST_F(OsdTelemetryTest, TimerCatchesUpAfterDriverStartAndFreezesOnDisarm)
{
	samples.status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	samples.status.takeoff_time = 2 * SECOND;
	samples.land.landed = false;
	update();
	EXPECT_EQ(core.data().flight_seconds, 8u);
	now += SECOND;
	refresh();
	samples.status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	samples.status.takeoff_time = 0;
	update();
	EXPECT_EQ(core.data().flight_seconds, 9u);
	now += SECOND;
	refresh();
	update();
	EXPECT_EQ(core.data().flight_seconds, 9u);
}

TEST_F(OsdTelemetryTest, ThrottleUsesOnlyFreshFiniteCommandedThrustAndClamps)
{
	samples.status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	samples.thrust[0].timestamp = now;
	samples.thrust[0].xyz[2] = -0.3f;
	samples.thrust[1].timestamp = now;
	samples.thrust[1].xyz[0] = 0.7f;
	update();
	EXPECT_NEAR(core.data().throttle_percent, 70.f, 1e-5f);
	samples.thrust[1].timestamp = now - SECOND;
	update();
	EXPECT_NEAR(core.data().throttle_percent, 30.f, 1e-5f);
	samples.thrust[0].xyz[2] = -2.f;
	update();
	EXPECT_FLOAT_EQ(core.data().throttle_percent, 100.f);
	samples.thrust[0].xyz[0] = INFINITY;
	update();
	EXPECT_TRUE(std::isnan(core.data().throttle_percent));
	samples.status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	update();
	EXPECT_FLOAT_EQ(core.data().throttle_percent, 0.f);
}

TEST_F(OsdTelemetryTest, VtolStoppedPusherDoesNotHideHoverThrust)
{
	samples.status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
	samples.thrust[0].timestamp = now;
	samples.thrust[0].xyz[0] = NAN; // Stopped pusher, as published by standard VTOL.
	samples.thrust[0].xyz[2] = -0.32f;
	samples.thrust[1].timestamp = now; // Unused second allocator publishes zeros.
	update();
	EXPECT_NEAR(core.data().throttle_percent, 32.f, 1e-5f);
	samples.thrust[0].xyz[1] = NAN;
	update();
	EXPECT_NEAR(core.data().throttle_percent, 32.f, 1e-5f);
	samples.thrust[0].xyz[2] = NAN;
	update();
	EXPECT_FLOAT_EQ(core.data().throttle_percent, 0.f); // All axes commanded stopped.
	samples.thrust[0].xyz[2] = INFINITY;
	samples.thrust[1].timestamp = now - SECOND;
	update();
	EXPECT_TRUE(std::isnan(core.data().throttle_percent));
}

TEST_F(OsdTelemetryTest, AutotuneInfoVisibleByDefaultAndDuplicateDoesNotRefreshExpiry)
{
	message("MC autotune: roll tuning", 6);
	update();
	EXPECT_STREQ(core.data().message, "MC autotune: roll tuning");
	const uint64_t original = now;
	now += 9 * SECOND;
	message("MC autotune: roll tuning", 6, original);
	update();
	EXPECT_STREQ(core.data().message, "MC autotune: roll tuning");
	now += SECOND;
	update();
	EXPECT_STREQ(core.data().message, "");
}

TEST_F(OsdTelemetryTest, NoneDisablesAllMessagesAndSeverityThresholdIsHonored)
{
	settings.log_level = 4;
	message("Informational message", 6);
	update();
	EXPECT_STREQ(core.data().message, "");
	++now;
	message("Warning message", 4);
	update();
	EXPECT_STREQ(core.data().message, "Warning message");
	settings.log_level = 8;
	update();
	EXPECT_STREQ(core.data().message, "");
	++now;
	message("Emergency", 0);
	update();
	EXPECT_STREQ(core.data().message, "");
	settings.log_level = 6;
	update();
	EXPECT_STREQ(core.data().message, "");
}

TEST_F(OsdTelemetryTest, HigherPriorityMessagePreemptsAndSurvivesInfoFlood)
{
	message("Progress", 6);
	update();
	EXPECT_STREQ(core.data().message, "Progress");
	++now;
	message("Failsafe", 2);
	update();
	EXPECT_STREQ(core.data().message, "Failsafe");

	for (unsigned i = 0; i < 20; ++i) {
		++now;
		message("More progress", 6);
	}

	update();
	EXPECT_STREQ(core.data().message, "Failsafe");
}

TEST_F(OsdTelemetryTest, MessagesScrollWithinThirtyColumnsAndDwellAtEnds)
{
	const char *text = "012345678901234567890123456789ABCDEF";
	message(text);
	update();
	EXPECT_EQ(strlen(core.data().message), 30u);
	EXPECT_STREQ(core.data().message, "012345678901234567890123456789");
	now += 500000;
	update();
	EXPECT_EQ(core.data().message[0], '0');
	now += 125000;
	update();
	EXPECT_EQ(core.data().message[0], '1');
	now += 625000;
	update();
	EXPECT_STREQ(core.data().message, "678901234567890123456789ABCDEF");
	now += 250000;
	update();
	EXPECT_STREQ(core.data().message, "678901234567890123456789ABCDEF");
}

TEST_F(OsdTelemetryTest, UnterminatedMessageIsBoundedAndControlsAreSanitized)
{
	mavlink_log_s log{};
	log.timestamp = now;
	log.severity = 4;
	memset(log.text, 'X', sizeof(log.text));
	log.text[5] = '\n';
	core.pushMessage(now, log, settings);
	update();
	EXPECT_EQ(strlen(core.data().message), 30u);
	EXPECT_EQ(core.data().message[5], ' ');
	now += 1000000;
	update();
	EXPECT_EQ(strlen(core.data().message), 30u);
}

TEST_F(OsdTelemetryTest, OldMessagesAndFutureTimestampsAreIgnored)
{
	message("Too old", 0, now - 10 * SECOND + 1);
	now += 1;
	update();
	EXPECT_STREQ(core.data().message, "");
	message("Future", 0, now + 1);
	update();
	EXPECT_STREQ(core.data().message, "");
	message("Current", 6);
	update();
	EXPECT_STREQ(core.data().message, "Current");
}

TEST_F(OsdTelemetryTest, AdapterConsumesPublishedBatteryStatusAndAutotuneMessages)
{
	// gtest_functional_main initializes the uORB manager. Unadvertise even if an assertion fails.
	struct Advertisement {
		orb_advert_t handle;
		~Advertisement()
		{
			if (handle != nullptr) {
				orb_unadvertise(handle);
			}
		}
	};

	mavlink_log_s log{};
	Advertisement battery_pub{orb_advertise(ORB_ID(battery_status), &samples.battery)};
	Advertisement status_pub{orb_advertise(ORB_ID(vehicle_status), &samples.status)};
	Advertisement log_pub{orb_advertise(ORB_ID(mavlink_log), &log)};
	ASSERT_NE(battery_pub.handle, nullptr);
	ASSERT_NE(status_pub.handle, nullptr);
	ASSERT_NE(log_pub.handle, nullptr);
	msp_osd::OsdTelemetry adapter;

	samples.battery.current_a = 23.f;
	samples.status.nav_state_display = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
	samples.status.is_vtol = true;
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	log.timestamp = now;
	log.severity = 6;
	strncpy(log.text, "FW autotune: sweep", sizeof(log.text) - 1);
	ASSERT_EQ(orb_publish(ORB_ID(battery_status), battery_pub.handle, &samples.battery), 0);
	ASSERT_EQ(orb_publish(ORB_ID(vehicle_status), status_pub.handle, &samples.status), 0);
	ASSERT_EQ(orb_publish(ORB_ID(mavlink_log), log_pub.handle, &log), 0);

	adapter.update(now, settings);
	EXPECT_FLOAT_EQ(adapter.data().battery_voltage, 24.f);
	EXPECT_FLOAT_EQ(adapter.data().compensated_battery_voltage, 25.2f);
	EXPECT_FLOAT_EQ(adapter.data().compensated_cell_voltage, 4.2f);
	EXPECT_FLOAT_EQ(adapter.data().battery_remaining_percent, 42.f);
	EXPECT_FLOAT_EQ(adapter.data().current_a, 23.f);
	EXPECT_STREQ(adapter.data().mode, "Return FW");
	EXPECT_STREQ(adapter.data().message, "FW autotune: sweep");

	// A new invalid battery update must clear the previous value; an unchanged log must expire.
	samples.battery.connected = false;
	samples.battery.timestamp = ++now;
	ASSERT_EQ(orb_publish(ORB_ID(battery_status), battery_pub.handle, &samples.battery), 0);
	adapter.update(now, settings);
	EXPECT_TRUE(std::isnan(adapter.data().current_a));
	EXPECT_TRUE(std::isnan(adapter.data().compensated_battery_voltage));
	EXPECT_TRUE(std::isnan(adapter.data().compensated_cell_voltage));
	EXPECT_TRUE(std::isnan(adapter.data().battery_remaining_percent));
	adapter.update(now + 10 * SECOND, settings);
	EXPECT_STREQ(adapter.data().message, "");
}

TEST_F(OsdTelemetryTest, FreshOutOfOrderWarningPreemptsNewerInfo)
{
	message("Newer progress", 6, now);
	update();
	EXPECT_STREQ(core.data().message, "Newer progress");
	message("Earlier stamped warning", 4, now - 1);
	update();
	EXPECT_STREQ(core.data().message, "Earlier stamped warning");
}

TEST_F(OsdTelemetryTest, SameTimestampDifferentMessageStartsItsOwnScrollingWindow)
{
	const uint64_t published = now;
	message("012345678901234567890123456789ABCDEF", 6, published);
	update();
	now += 750000;
	update();
	EXPECT_EQ(core.data().message[0], '2');
	message("WARNING 012345678901234567890123456789", 4, published);
	update();
	EXPECT_STREQ(core.data().message, "WARNING 0123456789012345678901");
}


TEST_F(OsdTelemetryTest, AirspeedSetpointFollowsControllerInMissionPositionAndAltitude)
{
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	samples.tecs.timestamp = now;
	samples.tecs.equivalent_airspeed_sp = 22.f;
	samples.tecs.true_airspeed_sp = 25.f;

	for (uint8_t mode : {
		     vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, vehicle_status_s::NAVIGATION_STATE_POSCTL,
		     vehicle_status_s::NAVIGATION_STATE_ALTCTL
	     }) {
		samples.status.nav_state = mode;
		update();
		EXPECT_FLOAT_EQ(core.data().airspeed_setpoint_m_s, 22.f);
		EXPECT_FLOAT_EQ(core.data().airspeed_m_s, 17.f);
	}

	samples.status.is_vtol = true;
	update();
	EXPECT_FLOAT_EQ(core.data().airspeed_setpoint_m_s, 22.f);
	samples.tecs.equivalent_airspeed_sp = 24.f;
	update();
	EXPECT_FLOAT_EQ(core.data().airspeed_setpoint_m_s, 24.f);
}

TEST_F(OsdTelemetryTest, AirspeedSetpointClearsOutsideFixedWingSpeedControlledModes)
{
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	samples.status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	samples.tecs.timestamp = now;
	samples.tecs.equivalent_airspeed_sp = 22.f;
	update();
	ASSERT_FLOAT_EQ(core.data().airspeed_setpoint_m_s, 22.f);

	for (uint8_t mode : {
		     vehicle_status_s::NAVIGATION_STATE_MANUAL, vehicle_status_s::NAVIGATION_STATE_ACRO,
		     vehicle_status_s::NAVIGATION_STATE_STAB, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
	     }) {
		samples.status.nav_state = mode;
		update();
		EXPECT_TRUE(std::isnan(core.data().airspeed_setpoint_m_s));
	}

	samples.status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	samples.status.is_vtol = true;
	samples.status.in_transition_mode = true;
	update();
	EXPECT_TRUE(std::isnan(core.data().airspeed_setpoint_m_s));
	samples.status.in_transition_mode = false;
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	update();
	EXPECT_TRUE(std::isnan(core.data().airspeed_setpoint_m_s));
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	samples.status.timestamp = now - 3 * SECOND;
	update();
	EXPECT_TRUE(std::isnan(core.data().airspeed_setpoint_m_s));
}

TEST_F(OsdTelemetryTest, AirspeedSetpointRejectsStaleInvalidAndPreviousModeSamples)
{
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	samples.status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	samples.tecs.equivalent_airspeed_sp = 22.f;

	for (uint64_t timestamp : {uint64_t(0), now + 1, now - SECOND / 2 - 1}) {
		samples.tecs.timestamp = timestamp;
		update();
		EXPECT_TRUE(std::isnan(core.data().airspeed_setpoint_m_s));
	}

	samples.tecs.timestamp = now;

	for (float invalid : {NAN, INFINITY, -1.f, 0.f}) {
		samples.tecs.equivalent_airspeed_sp = invalid;
		update();
		EXPECT_TRUE(std::isnan(core.data().airspeed_setpoint_m_s));
	}

	samples.tecs.equivalent_airspeed_sp = 22.f;
	samples.status.nav_state_timestamp = now;
	samples.tecs.timestamp = now - 1;
	update();
	EXPECT_TRUE(std::isnan(core.data().airspeed_setpoint_m_s));
	samples.tecs.timestamp = now;
	update();
	EXPECT_FLOAT_EQ(core.data().airspeed_setpoint_m_s, 22.f);
}

TEST_F(OsdTelemetryTest, AdapterConsumesPublishedAirspeedSetpoint)
{
	msp_osd::OsdTelemetry adapter;
	uORB::Publication<vehicle_status_s> status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<tecs_status_s> tecs_pub{ORB_ID(tecs_status)};
	uORB::Publication<fixed_wing_airspeed_status_s> airspeed_status_pub{ORB_ID(fixed_wing_airspeed_status)};
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	samples.status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	samples.tecs.timestamp = now;
	samples.tecs.equivalent_airspeed_sp = 22.f;
	samples.airspeed_status.timestamp = now;
	samples.airspeed_status.airspeed_override_active = true;
	ASSERT_TRUE(status_pub.publish(samples.status));
	ASSERT_TRUE(tecs_pub.publish(samples.tecs));
	ASSERT_TRUE(airspeed_status_pub.publish(samples.airspeed_status));
	adapter.update(now, settings);
	EXPECT_FLOAT_EQ(adapter.data().airspeed_setpoint_m_s, 22.f);
	EXPECT_TRUE(adapter.data().airspeed_override_active);
	samples.tecs.equivalent_airspeed_sp = 24.f;
	ASSERT_TRUE(tecs_pub.publish(samples.tecs));
	adapter.update(now, settings);
	EXPECT_FLOAT_EQ(adapter.data().airspeed_setpoint_m_s, 24.f);
	samples.airspeed_status.airspeed_override_active = false;
	ASSERT_TRUE(airspeed_status_pub.publish(samples.airspeed_status));
	adapter.update(now, settings);
	EXPECT_FALSE(adapter.data().airspeed_override_active);
	adapter.update(now + SECOND, settings);
	EXPECT_TRUE(std::isnan(adapter.data().airspeed_setpoint_m_s));
	EXPECT_FALSE(adapter.data().airspeed_override_active);
}

TEST_F(OsdTelemetryTest, OverrideRequiresExplicitFreshMissionStatus)
{
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	samples.status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	samples.status.nav_state_timestamp = now - SECOND;
	samples.tecs.timestamp = now;
	samples.tecs.equivalent_airspeed_sp = 22.f;
	samples.airspeed_status.timestamp = now;
	update();
	EXPECT_FALSE(core.data().airspeed_override_active);
	samples.tecs.equivalent_airspeed_sp = 26.f; // Commands, turn limits and slew alone are not an override.
	update();
	EXPECT_FALSE(core.data().airspeed_override_active);
	samples.airspeed_status.airspeed_override_active = true;
	update();
	ASSERT_TRUE(core.data().airspeed_override_active);
	samples.status.is_vtol = true;
	update();
	EXPECT_TRUE(core.data().airspeed_override_active);

	for (uint64_t timestamp : {uint64_t(0), now + 1, now - SECOND / 2 - 1}) {
		samples.airspeed_status.timestamp = timestamp;
		update();
		EXPECT_FALSE(core.data().airspeed_override_active);
		EXPECT_FLOAT_EQ(core.data().airspeed_setpoint_m_s, 26.f);
	}

	samples.airspeed_status.timestamp = now - 1;
	samples.status.nav_state_timestamp = now;
	update();
	EXPECT_FALSE(core.data().airspeed_override_active);
	samples.airspeed_status.timestamp = now;
	update();
	EXPECT_TRUE(core.data().airspeed_override_active);
	samples.airspeed_status.airspeed_override_active = false;
	update();
	EXPECT_FALSE(core.data().airspeed_override_active);
}

TEST_F(OsdTelemetryTest, OverrideClearsWithModeTransitionOrUnavailableTarget)
{
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	samples.status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	samples.tecs.timestamp = samples.airspeed_status.timestamp = now;
	samples.tecs.equivalent_airspeed_sp = 22.f;
	samples.airspeed_status.airspeed_override_active = true;
	update();
	ASSERT_TRUE(core.data().airspeed_override_active);

	for (uint8_t mode : {
		     vehicle_status_s::NAVIGATION_STATE_POSCTL, vehicle_status_s::NAVIGATION_STATE_ALTCTL,
		     vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, vehicle_status_s::NAVIGATION_STATE_MANUAL
	     }) {
		samples.status.nav_state = mode;
		update();
		EXPECT_FALSE(core.data().airspeed_override_active);
	}

	samples.status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	samples.status.in_transition_mode = true;
	update();
	EXPECT_FALSE(core.data().airspeed_override_active);
	samples.status.in_transition_mode = false;
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	update();
	EXPECT_FALSE(core.data().airspeed_override_active);
	samples.status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	samples.status.timestamp = now - 3 * SECOND;
	update();
	EXPECT_FALSE(core.data().airspeed_override_active);
	samples.status.timestamp = now;
	samples.tecs.timestamp = now - SECOND;
	update();
	EXPECT_FALSE(core.data().airspeed_override_active);
	samples.tecs.timestamp = now;

	for (float invalid : {NAN, INFINITY, -1.f, 0.f}) {
		samples.tecs.equivalent_airspeed_sp = invalid;
		update();
		EXPECT_FALSE(core.data().airspeed_override_active);
	}
}
