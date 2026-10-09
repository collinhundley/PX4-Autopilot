/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include <gtest/gtest.h>
#include <hrt_work.h>
#include <memory>

#include "FixedWingModeManager.hpp"

class MissionAirspeedTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		hrt_init();
		hrt_work_queue_init();
		px4_log_initialize();
	}

	template<typename T> void parameter(const char *name, T value)
	{
		ASSERT_EQ(param_set_no_notification(param_find(name), &value), 0);

		if (_manager) {
			_manager->parameters_update();
		}
	}

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
		parameter("FW_AIRSPD_MIN", 10.f);
		parameter("FW_AIRSPD_TRIM", 15.f);
		parameter("FW_AIRSPD_MAX", 30.f);
		_manager = std::make_unique<FixedWingModeManager>();

		_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
		_control.flag_armed = true;
		_control.flag_control_auto_enabled = true;
		_control.flag_control_position_enabled = true;

		_local.xy_valid = _local.z_valid = _local.v_xy_valid = _local.v_z_valid = true;
		_local.xy_global = _local.z_global = true;
		_local.ref_lat = 47.;
		_local.ref_lon = 8.;
		_local.ref_alt = 500.f;
		_local.z = -100.f;
		_local.vx = 18.f;

		vehicle_global_position_s global{};
		global.lat = _local.ref_lat;
		global.lon = _local.ref_lon;
		global.alt = 600.f;
		_global_pub.publish(global);
		vehicle_attitude_s attitude{};
		attitude.q[0] = 1.f;
		_attitude_pub.publish(attitude);
		vehicle_land_detected_s land{};
		_land_pub.publish(land);

		_triplet.previous.valid = true;
		_triplet.previous.lat = 47.;
		_triplet.previous.lon = 8.;
		_triplet.previous.alt = 600.f;
		_triplet.current = _triplet.previous;
		_triplet.current.lat = 47.001;
		_triplet.current.course = NAN;
		_triplet.current.cruising_speed = 18.f;
		_triplet.current.loiter_radius = 80.f;
		_triplet.next.lat = _triplet.next.lon = NAN;
		_triplet.next.alt = NAN;
		_triplet_pub.publish(_triplet);
		sticks(0.f);
	}

	void sticks(float throttle, bool valid = true, float pitch = 0.f)
	{
		manual_control_setpoint_s input{};
		input.timestamp = hrt_absolute_time();
		input.valid = valid;
		input.data_source = manual_control_setpoint_s::SOURCE_RC;
		input.throttle = throttle;
		input.pitch = pitch;
		_stick_pub.publish(input);
	}

	void mission(float speed, uint8_t type = position_setpoint_s::SETPOINT_TYPE_POSITION)
	{
		_triplet.current.cruising_speed = speed;
		_triplet.current.type = type;
		_triplet_pub.publish(_triplet);
	}

	bool run(fixed_wing_longitudinal_setpoint_s &output)
	{
		// Match the module's 50 Hz local-position subscription interval.
		px4_usleep(22000);
		_local.timestamp = hrt_absolute_time();
		_status.timestamp = _control.timestamp = _local.timestamp;
		_status_pub.publish(_status);
		_control_pub.publish(_control);
		_local_pub.publish(_local);
		_manager->Run();
		EXPECT_TRUE(_airspeed_status_sub.update(&_airspeed_status));
		return _output_sub.update(&output);
	}

	float airspeed()
	{
		fixed_wing_longitudinal_setpoint_s output{};
		EXPECT_TRUE(run(output));
		return output.equivalent_airspeed;
	}

	float storedSpeed() { return _manager->_pos_sp_triplet.current.cruising_speed; }
	bool overrideActive() { return _airspeed_status.airspeed_override_active; }
	void abortLanding() { _manager->updateLandingAbortStatus(position_controller_landing_status_s::ABORTED_BY_OPERATOR); }

	void commandSpeed(float speed)
	{
		vehicle_command_s command{};
		command.timestamp = hrt_absolute_time();
		command.command = vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED;
		command.param1 = vehicle_command_s::SPEED_TYPE_AIRSPEED;
		command.param2 = speed;
		_command_pub.publish(command);
	}

	std::unique_ptr<FixedWingModeManager> _manager;
	vehicle_status_s _status{};
	vehicle_control_mode_s _control{};
	vehicle_local_position_s _local{};
	position_setpoint_triplet_s _triplet{};
	uORB::Publication<vehicle_status_s> _status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_control_mode_s> _control_pub{ORB_ID(vehicle_control_mode)};
	uORB::Publication<vehicle_local_position_s> _local_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_global_position_s> _global_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_land_detected_s> _land_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<position_setpoint_triplet_s> _triplet_pub{ORB_ID(position_setpoint_triplet)};
	uORB::Publication<manual_control_setpoint_s> _stick_pub{ORB_ID(manual_control_setpoint)};
	uORB::Publication<vehicle_command_s> _command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<failsafe_flags_s> _failsafe_pub{ORB_ID(failsafe_flags)};
	uORB::Subscription _output_sub{ORB_ID(fixed_wing_longitudinal_setpoint)};
	uORB::Subscription _airspeed_status_sub{ORB_ID(fixed_wing_airspeed_status)};
	fixed_wing_airspeed_status_s _airspeed_status{};
};

TEST_F(MissionAirspeedTest, DisabledByDefaultAndCanBeDisabledInFlight)
{
	int32_t enabled = -1;
	ASSERT_EQ(param_get(param_find("FW_MIS_THR_NUDGE"), &enabled), 0);
	EXPECT_EQ(enabled, 0);
	sticks(1.f);
	EXPECT_FLOAT_EQ(airspeed(), 18.f);
	EXPECT_FALSE(overrideActive());
	mission(NAN);
	EXPECT_TRUE(std::isnan(airspeed()));
	parameter("FW_MIS_THR_NUDGE", int32_t(1));
	EXPECT_FLOAT_EQ(airspeed(), 30.f);
	EXPECT_TRUE(overrideActive());
	parameter("FW_MIS_THR_NUDGE", int32_t(0));
	EXPECT_TRUE(std::isnan(airspeed()));
	EXPECT_FALSE(overrideActive());
}

TEST_F(MissionAirspeedTest, MapsThrottleAroundMissionCommandWithoutAccumulating)
{
	parameter("FW_MIS_THR_NUDGE", int32_t(1));
	const float throttle[] = {-1.f, -.53f, 0.f, .53f, 1.f, .53f, .53f, 0.f};
	const float expected[] = {10.f, 14.f, 18.f, 24.f, 30.f, 24.f, 24.f, 18.f};

	for (unsigned i = 0; i < sizeof(throttle) / sizeof(throttle[0]); ++i) {
		sticks(throttle[i]);
		EXPECT_FLOAT_EQ(airspeed(), expected[i]);
		EXPECT_FLOAT_EQ(storedSpeed(), 18.f);
		EXPECT_EQ(overrideActive(), fabsf(throttle[i]) > 0.f);
	}
}

TEST_F(MissionAirspeedTest, MissingSpeedUsesTrimAndCommandIsConstrained)
{
	parameter("FW_MIS_THR_NUDGE", int32_t(1));

	for (float unset : {NAN, -1.f, 0.f, INFINITY}) {
		mission(unset);
		sticks(0.f);
		EXPECT_FLOAT_EQ(airspeed(), 15.f);
		sticks(-.53f);
		EXPECT_FLOAT_EQ(airspeed(), 12.5f);
		sticks(.53f);
		EXPECT_FLOAT_EQ(airspeed(), 22.5f);
	}

	sticks(0.f);
	mission(5.f);
	EXPECT_FLOAT_EQ(airspeed(), 10.f);
	mission(40.f);
	EXPECT_FLOAT_EQ(airspeed(), 30.f);
}

TEST_F(MissionAirspeedTest, NewSpeedCommandUpdatesReferenceWhileDeflected)
{
	parameter("FW_MIS_THR_NUDGE", int32_t(1));
	sticks(.53f);
	EXPECT_FLOAT_EQ(airspeed(), 24.f);
	commandSpeed(22.f);
	EXPECT_FLOAT_EQ(airspeed(), 26.f);
	EXPECT_TRUE(overrideActive());
	EXPECT_FLOAT_EQ(storedSpeed(), 22.f);
	sticks(0.f);
	EXPECT_FLOAT_EQ(airspeed(), 22.f);
	EXPECT_FALSE(overrideActive());
	// Navigator carries the commanded speed forward to subsequent waypoints.
	mission(22.f, position_setpoint_s::SETPOINT_TYPE_LOITER);
	EXPECT_FLOAT_EQ(airspeed(), 22.f);
	mission(-1.f);
	EXPECT_FLOAT_EQ(airspeed(), 15.f);
}

TEST_F(MissionAirspeedTest, RcLossAndNonfiniteInputRestoreMissionSpeed)
{
	parameter("FW_MIS_THR_NUDGE", int32_t(1));
	sticks(1.f);
	EXPECT_FLOAT_EQ(airspeed(), 30.f);
	sticks(1.f, false);
	EXPECT_FLOAT_EQ(airspeed(), 18.f);
	EXPECT_FALSE(overrideActive());
	sticks(NAN);
	EXPECT_FLOAT_EQ(airspeed(), 18.f);
	EXPECT_FALSE(overrideActive());
	sticks(1.f);
	EXPECT_FLOAT_EQ(airspeed(), 30.f);
	failsafe_flags_s failsafe{};
	failsafe.manual_control_signal_lost = true;
	_failsafe_pub.publish(failsafe);
	EXPECT_FLOAT_EQ(airspeed(), 18.f);
	EXPECT_FALSE(overrideActive());
	mission(NAN);
	EXPECT_TRUE(std::isnan(airspeed()));
	sticks(0.f);
	EXPECT_FLOAT_EQ(airspeed(), 15.f);
}

TEST_F(MissionAirspeedTest, OtherAutoModesIgnoreThrottle)
{
	parameter("FW_MIS_THR_NUDGE", int32_t(1));
	sticks(1.f);

	for (uint8_t mode : {
		     vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL,
		     vehicle_status_s::NAVIGATION_STATE_OFFBOARD
	     }) {
		_status.nav_state = mode;
		EXPECT_FLOAT_EQ(airspeed(), 18.f);
		EXPECT_FALSE(overrideActive());
	}

	_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	EXPECT_FLOAT_EQ(airspeed(), 30.f);
}

TEST_F(MissionAirspeedTest, VtolFixedWingOnly)
{
	parameter("FW_MIS_THR_NUDGE", int32_t(1));
	_status.is_vtol = _status.is_vtol_tailsitter = true;
	sticks(-1.f);
	EXPECT_FLOAT_EQ(airspeed(), 10.f);
	EXPECT_TRUE(overrideActive());
	_status.in_transition_mode = _status.in_transition_to_fw = true;
	EXPECT_FLOAT_EQ(airspeed(), 18.f);
	EXPECT_FALSE(overrideActive());
	_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	EXPECT_FLOAT_EQ(airspeed(), 18.f);
	_status.in_transition_mode = _status.in_transition_to_fw = false;
	fixed_wing_longitudinal_setpoint_s output{};
	EXPECT_FALSE(run(output));
	EXPECT_FALSE(overrideActive());
	_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	EXPECT_FLOAT_EQ(airspeed(), 10.f);
}

TEST_F(MissionAirspeedTest, MissionAlwaysUsesThrottleAndLoiterIsSupported)
{
	parameter("FW_MIS_THR_NUDGE", int32_t(1));
	parameter("FW_POS_STK_CONF", int32_t(3));
	sticks(.53f, true, -1.f);
	mission(18.f, position_setpoint_s::SETPOINT_TYPE_LOITER);
	EXPECT_FLOAT_EQ(airspeed(), 24.f);
	_triplet.current.course = .5f;
	_triplet_pub.publish(_triplet);
	EXPECT_FLOAT_EQ(airspeed(), 24.f);
#ifdef CONFIG_FIGURE_OF_EIGHT
	_triplet.current.course = NAN;
	_triplet.current.loiter_pattern = position_setpoint_s::LOITER_TYPE_FIGUREEIGHT;
	_triplet.current.loiter_minor_radius = 40.f;
	_triplet_pub.publish(_triplet);
	EXPECT_FLOAT_EQ(airspeed(), 24.f);
#endif
}

TEST_F(MissionAirspeedTest, TakeoffAndLandingRetainDedicatedAirspeeds)
{
	parameter("FW_MIS_THR_NUDGE", int32_t(1));
	parameter("FW_TKO_AIRSPD", 13.f);
	parameter("FW_LND_AIRSPD", 12.f);
	parameter("FW_LND_USETER", int32_t(0));
	sticks(1.f);
	mission(18.f, position_setpoint_s::SETPOINT_TYPE_TAKEOFF);
	EXPECT_FLOAT_EQ(airspeed(), 13.f);
	EXPECT_FALSE(overrideActive());
	mission(18.f, position_setpoint_s::SETPOINT_TYPE_LAND);
	EXPECT_FLOAT_EQ(airspeed(), 12.f);
	EXPECT_FALSE(overrideActive());
}

TEST_F(MissionAirspeedTest, EarlyLandingConfigurationTakesPriority)
{
	parameter("FW_MIS_THR_NUDGE", int32_t(1));
	parameter("FW_LND_EARLYCFG", int32_t(1));
	parameter("FW_LND_AIRSPD", 12.f);
	sticks(1.f);
	_triplet.current.lat = _local.ref_lat;
	_triplet.next = _triplet.current;
	_triplet.next.type = position_setpoint_s::SETPOINT_TYPE_LAND;
	mission(18.f, position_setpoint_s::SETPOINT_TYPE_LOITER);
	EXPECT_FLOAT_EQ(airspeed(), 12.f);
	EXPECT_FALSE(overrideActive());
	// Also preserve the existing loiter speed when no explicit landing speed is configured.
	parameter("FW_LND_AIRSPD", -1.f);
	EXPECT_FLOAT_EQ(airspeed(), 18.f);
}

TEST_F(MissionAirspeedTest, LandingAbortRetainsMissionSpeedUntilClearance)
{
	parameter("FW_MIS_THR_NUDGE", int32_t(1));
	parameter("FW_LND_ABORT", int32_t(1));
	sticks(1.f);
	_triplet.current.alt = 650.f;
	mission(18.f, position_setpoint_s::SETPOINT_TYPE_LOITER);
	EXPECT_FLOAT_EQ(airspeed(), 30.f);
	abortLanding();
	EXPECT_FLOAT_EQ(airspeed(), 18.f);
	EXPECT_FALSE(overrideActive());
	_local.z = -150.f;
	EXPECT_FLOAT_EQ(airspeed(), 18.f);
	EXPECT_FLOAT_EQ(airspeed(), 30.f);
	EXPECT_TRUE(overrideActive());
}

TEST_F(MissionAirspeedTest, DefaultDeadbandHoldsReferenceFrom47To53Percent)
{
	parameter("FW_MIS_THR_NUDGE", int32_t(1));
	float deadband = -1.f;
	ASSERT_EQ(param_get(param_find("FW_MIS_THR_DZ"), &deadband), 0);
	EXPECT_FLOAT_EQ(deadband, 3.f);

	for (float speed : {18.f, NAN}) {
		mission(speed);
		const float reference = std::isfinite(speed) ? speed : 15.f;

		for (float throttle : {-.06f, -.04f, 0.f, .04f, .06f}) {
			// Normalized [-1, 1] inputs correspond to 47%, 48%, 50%, 52%, 53% travel.
			sticks(throttle);
			EXPECT_FLOAT_EQ(airspeed(), reference);
			EXPECT_FALSE(overrideActive());
		}

		for (float throttle : {-.06001f, .06001f}) {
			sticks(throttle);
			const float target = airspeed();
			EXPECT_NEAR(target, reference, .0002f); // Continuous on both sides of the deadband.
			EXPECT_EQ(target > reference, throttle > 0.f);
			EXPECT_TRUE(overrideActive());
		}
	}

	sticks(-.04f); // A new command at 48% does not activate the override.
	commandSpeed(22.f);
	EXPECT_FLOAT_EQ(airspeed(), 22.f);
	EXPECT_FALSE(overrideActive());
}

TEST_F(MissionAirspeedTest, DeadbandCanBeChangedOrDisabledWithoutLosingEndpoints)
{
	parameter("FW_MIS_THR_NUDGE", int32_t(1));

	for (float deadband : {0.f, 10.f, 25.f}) {
		parameter("FW_MIS_THR_DZ", deadband);
		const float edge = deadband / 50.f;

		for (float throttle : {-edge, 0.f, edge}) {
			sticks(throttle);
			EXPECT_FLOAT_EQ(airspeed(), 18.f);
			EXPECT_FALSE(overrideActive());
		}

		sticks(-(1.f + edge) / 2.f);
		EXPECT_FLOAT_EQ(airspeed(), 14.f);
		EXPECT_TRUE(overrideActive());
		sticks((1.f + edge) / 2.f);
		EXPECT_FLOAT_EQ(airspeed(), 24.f);
		EXPECT_TRUE(overrideActive());
		sticks(-1.f);
		EXPECT_FLOAT_EQ(airspeed(), 10.f);
		sticks(1.f);
		EXPECT_FLOAT_EQ(airspeed(), 30.f);
	}
}
