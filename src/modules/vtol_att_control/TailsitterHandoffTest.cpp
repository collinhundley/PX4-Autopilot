/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
#include <memory>
#include <hrt_work.h>
#include "vtol_att_control_main.h"
#include "../fw_rate_control/FixedwingRateControl.hpp"
#include "../fw_att_control/FixedwingAttitudeControl.hpp"

using matrix::Vector3f;

// Real controller, uORB handshake and motor routing, invoked synchronously without work queues.
class TailsitterHandoffTest : public ::testing::Test
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
	}

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
		parameter("VT_TYPE", int32_t(0));
		parameter("VT_TS_THR_SLEW", .2f);
		parameter("VT_FW_DIFTHR_EN", int32_t(7));
		parameter("VT_FW_DIFTHR_S_Y", .7f);
		parameter("VT_FW_DIFTHR_S_P", .8f);
		parameter("VT_FW_DIFTHR_S_R", .6f);
		parameter("FW_ARSP_SCALE_EN", int32_t(0));
		parameter("TRIM_ROLL", .03f);
		parameter("TRIM_PITCH", -.02f);
		parameter("TRIM_YAW", .01f);
		parameter("FW_RLL_TO_YAW_FF", .3f);
		_fw = std::make_unique<FixedwingRateControl>(true);
		_router = std::make_unique<VtolAttitudeControl>();
		_tailsitter = std::make_unique<Tailsitter>(_router.get());
		vehicle_land_detected_s land{};
		_land.publish(land);
		mode(true);
	}

	void mode(bool fw)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.is_vtol = status.is_vtol_tailsitter = true;
		status.vehicle_type = fw ? vehicle_status_s::VEHICLE_TYPE_FIXED_WING : vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		_vehicle.publish(status);
		vehicle_control_mode_s control{};
		control.timestamp = status.timestamp;
		control.flag_armed = control.flag_control_rates_enabled = control.flag_control_attitude_enabled = true;
		_control.publish(control);
		*_router->get_control_mode() = control;
		_router->get_land_detected()->landed = false;
	}

	void begin(float thrust = .37f, float sign = 1.f)
	{
		mode(true);
		_mc = {};
		_mc.timestamp_sample = hrt_absolute_time();
		_mc.timestamp = _mc.timestamp_sample;
		_mc.thrust = thrust;
		Vector3f(.12f, -.18f, .09f).copyTo(_mc.torque);
		(sign * Vector3f(.04f, -.09f, .06f)).copyTo(_mc.torque_bias);
		Vector3f(.1f, -.2f, .3f).copyTo(_mc.rates);
		_mc_state.publish(_mc);
		Vector3f(_mc.torque).copyTo(_router->get_torque_setpoint_0()->xyz);
		_router->get_thrust_setpoint_0()->xyz[2] = -thrust;
		_tailsitter->_vtol_mode = Tailsitter::vtol_mode::FW_MODE;
		_tailsitter->_trans_finished_ts = hrt_absolute_time();
		_tailsitter->startHandoff();
		px4_usleep(1);
	}

	void demand(float throttle, bool fresh = true)
	{
		vehicle_rates_setpoint_s sp{};
		sp.timestamp = fresh ? hrt_absolute_time() : 1;
		sp.roll = -.3f;
		sp.pitch = .4f;
		sp.yaw = -.2f;
		sp.thrust_body[0] = throttle;
		_rates.publish(sp);
	}

	void runFw()
	{
		vehicle_angular_velocity_s angular{};
		angular.timestamp = angular.timestamp_sample = hrt_absolute_time();
		Vector3f(.05f, -.07f, .09f).copyTo(angular.xyz);
		Vector3f(.2f, -.3f, .4f).copyTo(angular.xyz_derivative);
		_angular.publish(angular);
		_fw->Run();
		_fw->ScheduleClear();
	}

	void route(bool torque = true, bool thrust = true)
	{
		if (torque) { _torque.copy(_router->get_vehicle_torque_setpoint_virtual_fw()); }

		if (thrust) { _thrust.copy(_router->get_vehicle_thrust_setpoint_virtual_fw()); }

		_tailsitter->fill_actuator_outputs();
	}

	void expectHeld()
	{
		EXPECT_LT((Vector3f(_router->get_torque_setpoint_0()->xyz) - Vector3f(_mc.torque)).norm(), 1e-6f);
		EXPECT_NEAR(_router->get_thrust_setpoint_0()->xyz[2], -_mc.thrust, 1e-6f);
	}

	bool active() const { return _tailsitter->_handoff.active; }
	bool throttleLimited() const { return _fw->_throttle_slew.active(); }
	uint64_t initialized() const { return _fw->_handoff_initialized; }
	float directYawTrim() const { return _fw->_handoff_trim(2); }
	Vector3f bias() const
	{
		Vector3f fw = _fw->_gain_compression.getGains().emult(_fw->_rate_control.getIntegral())
			      + _fw->_handoff_trim + Vector3f(.03f, -.02f, .01f);
		fw(2) += .3f * fw(0);
		return tailsitter_handoff::toMC(fw).emult(Vector3f(.7f, .8f, .6f));
	}

	void abort()
	{
		// The router's default request is MC. Exercise the real FW -> back transition cancellation.
		_tailsitter->update_vtol_state();
		EXPECT_EQ(_tailsitter->_handoff.handoff_id, 0u);
		mode(false);
		runFw();
	}

	void expireOutputs()
	{
		_router->get_vehicle_torque_setpoint_virtual_fw()->timestamp = 0;
		_router->get_vehicle_thrust_setpoint_virtual_fw()->timestamp = 0;
		_tailsitter->fill_actuator_outputs();
	}

	void expireHandoff()
	{
		_tailsitter->_last_valid_fw_output = hrt_absolute_time() - tailsitter_handoff::kOutputTimeout - 1;
		expireOutputs();
	}

	void saturatedAllocator()
	{
		uORB::Publication<control_allocator_status_s> allocation{ORB_ID(control_allocator_status)};
		control_allocator_status_s status{};
		status.timestamp = hrt_absolute_time();
		Vector3f(.1f, -.1f, .1f).copyTo(status.unallocated_torque);
		allocation.publish(status);
		const Vector3f before = _fw->_rate_control.getIntegral();
		runFw();
		EXPECT_EQ(_fw->_rate_control.getIntegral(), before);
		// These hover-frame residuals must inhibit negative FW roll/pitch and positive FW yaw.
		EXPECT_EQ(_fw->_allocator_saturation, Vector3f(-.1f, -.1f, .1f));
	}

	void directYaw()
	{
		vehicle_control_mode_s control = *_router->get_control_mode();
		control.flag_control_attitude_enabled = false;
		control.flag_control_manual_enabled = true;
		_control.publish(control);
		*_router->get_control_mode() = control;
		parameter("FW_ACRO_YAW_EN", int32_t(0));
		_fw->updateParams();
		manual_control_setpoint_s manual{};
		manual.timestamp = hrt_absolute_time();
		manual.valid = true;
		manual.throttle = 0.f;
		_pilot.publish(manual);
	}

	void surfaceOnlyRoll()
	{
		parameter("VT_FW_DIFTHR_EN", int32_t(6));
		_fw->parameters_update();
		_tailsitter->updateParams();
	}

	void batteryScale(float scale)
	{
		parameter("FW_BAT_SCALE_EN", int32_t(1));
		_fw->updateParams();
		_fw->parameters_update();
		ASSERT_TRUE(_fw->_param_fw_bat_scale_en.get());
		battery_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.connected = true;
		status.scale = scale;
		ASSERT_TRUE(_battery.publish(status));
		ASSERT_TRUE(_fw->_battery_status_sub.updated());
	}

	void attitudeSourceReadiness()
	{
		FixedwingAttitudeControl attitude(true);
		vehicle_control_mode_s control = *_router->get_control_mode();
		control.flag_control_altitude_enabled = true;
		_control.publish(control);
		vehicle_attitude_setpoint_s sp{};
		sp.q_d[0] = 1.f;
		sp.thrust_body[0] = .6f;
		sp.timestamp = _tailsitter->_handoff.handoff_id - 1;
		uORB::Publication<vehicle_attitude_setpoint_s> source{ORB_ID(vehicle_attitude_setpoint)};
		uORB::Publication<vehicle_attitude_s> orientation{ORB_ID(vehicle_attitude)};
		uORB::Publication<tecs_status_s> tecs_pub{ORB_ID(tecs_status)};
		uORB::Subscription rates_sub{ORB_ID(vehicle_rates_setpoint_virtual_fw)};
		vehicle_rates_setpoint_s ignored{};
		rates_sub.copy(&ignored);
		const auto run = [&]() {
			vehicle_attitude_s att{};
			att.timestamp = hrt_absolute_time();
			Quatf(matrix::Eulerf(0.f, -M_PI_2_F, 0.f)).copyTo(att.q);
			orientation.publish(att);
			source.publish(sp);
			attitude.Run();
			attitude.ScheduleClear();
		};
		tecs_status_s tecs{};
		tecs.timestamp = sp.timestamp;
		tecs_pub.publish(tecs);
		run();
		EXPECT_FALSE(rates_sub.updated());
		tecs.timestamp = hrt_absolute_time();
		tecs_pub.publish(tecs);
		run(); // new TECS status alone is insufficient: target still predates the handoff
		EXPECT_FALSE(rates_sub.updated());
		sp.timestamp = hrt_absolute_time();
		run();
		ASSERT_TRUE(rates_sub.update(&ignored));
		EXPECT_FLOAT_EQ(ignored.thrust_body[0], sp.thrust_body[0]);
		runFw();
		route();
	}

	void manualAttitudeSource()
	{
		FixedwingAttitudeControl attitude(true);
		vehicle_control_mode_s control = *_router->get_control_mode();
		control.flag_control_manual_enabled = true;
		_control.publish(control);
		manual_control_setpoint_s manual{};
		manual.timestamp = hrt_absolute_time();
		manual.valid = true;
		manual.throttle = -.3f; // 35% stick, deliberately different from outgoing MC thrust
		_pilot.publish(manual);
		vehicle_attitude_setpoint_s old_mc{};
		old_mc.timestamp = _tailsitter->_handoff.handoff_id - 1;
		old_mc.q_d[0] = 1.f;
		old_mc.thrust_body[2] = -.37f;
		uORB::Publication<vehicle_attitude_setpoint_s> source{ORB_ID(vehicle_attitude_setpoint)};
		source.publish(old_mc);
		uORB::Publication<vehicle_attitude_s> orientation{ORB_ID(vehicle_attitude)};
		uORB::Subscription rates_sub{ORB_ID(vehicle_rates_setpoint_virtual_fw)};

		for (int i = 0; i < 2; ++i) {
			vehicle_attitude_s att{};
			att.timestamp = att.timestamp_sample = hrt_absolute_time();
			Quatf(matrix::Eulerf(0.f, -M_PI_2_F, 0.f)).copyTo(att.q);
			orientation.publish(att);
			attitude.Run();
			attitude.ScheduleClear();
		}

		vehicle_rates_setpoint_s sp{};
		ASSERT_TRUE(rates_sub.update(&sp));
		EXPECT_FLOAT_EQ(sp.thrust_body[0], .35f);
		EXPECT_FLOAT_EQ(sp.thrust_body[2], 0.f);
		runFw();
		route();
	}

	std::unique_ptr<FixedwingRateControl> _fw;
	std::unique_ptr<VtolAttitudeControl> _router;
	std::unique_ptr<Tailsitter> _tailsitter;
	tailsitter_handoff_s _mc{};
	uORB::Publication<tailsitter_handoff_s> _mc_state{ORB_ID(tailsitter_handoff_mc)};
	uORB::Publication<vehicle_status_s> _vehicle{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_control_mode_s> _control{ORB_ID(vehicle_control_mode)};
	uORB::Publication<vehicle_land_detected_s> _land{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_rates_setpoint_s> _rates{ORB_ID(vehicle_rates_setpoint_virtual_fw)};
	uORB::Publication<vehicle_angular_velocity_s> _angular{ORB_ID(vehicle_angular_velocity)};
	uORB::Publication<manual_control_setpoint_s> _pilot{ORB_ID(manual_control_setpoint)};
	uORB::Publication<battery_status_s> _battery{ORB_ID(battery_status)};
	uORB::Subscription _torque{ORB_ID(vehicle_torque_setpoint_virtual_fw)};
	uORB::Subscription _thrust{ORB_ID(vehicle_thrust_setpoint_virtual_fw)};
};

TEST_F(TailsitterHandoffTest, WaitsBeyond50msForBothMatchedPublications)
{
	begin();
	demand(.8f, false);
	runFw();
	route();
	EXPECT_FALSE(active());
	EXPECT_EQ(initialized(), 0u);
	expectHeld();
	px4_usleep(60000);
	route();
	expectHeld();
	EXPECT_FALSE(active());
	demand(.8f);
	runFw();
	route(true, false);
	EXPECT_FALSE(active());
	expectHeld();
	route();
	EXPECT_TRUE(active());
	expectHeld();
	runFw();
	route();
	expectHeld();
}

TEST_F(TailsitterHandoffTest, TransfersAllBiasAxesWithoutWritingTrimParameters)
{
	begin();
	demand(.6f);
	runFw();
	EXPECT_LT((bias() - Vector3f(_mc.torque_bias)).norm(), 1e-6f);
	float trim = 0.f;
	param_get(param_find("TRIM_PITCH"), &trim);
	EXPECT_FLOAT_EQ(trim, -.02f);
	route();
	expectHeld();
}

TEST_F(TailsitterHandoffTest, ArbitraryThrottleAndRapidIncreaseUseHandoffSlew)
{
	for (float stick : {0.f, .1f, .5f, .9f, 1.f}) {
		begin();
		demand(stick);
		runFw();
		route();
		ASSERT_TRUE(active());
		expectHeld();
		runFw();
		route();
		expectHeld();

		if (stick < 1.f) {
			demand(1.f);
			runFw();
			route();
			EXPECT_GT(_router->get_thrust_setpoint_0()->xyz[2], -.38f);
			EXPECT_LT(_router->get_thrust_setpoint_0()->xyz[2], -.37f);
		}

		abort();
	}
}

TEST_F(TailsitterHandoffTest, CompletedSlewDoesNotLimitLaterFwDemands)
{
	begin(.37f);
	demand(.36f);
	runFw();
	route();
	ASSERT_TRUE(active());
	ASSERT_TRUE(throttleLimited());

	for (int i = 0; i < 40; ++i) {
		demand(.36f);
		runFw();
		route();
	}

	ASSERT_FALSE(throttleLimited());
	EXPECT_NEAR(_router->get_thrust_setpoint_0()->xyz[2], -.36f, 1e-6f);
	demand(.9f);
	runFw();
	route();
	EXPECT_NEAR(_router->get_thrust_setpoint_0()->xyz[2], -.9f, 1e-6f);
	demand(.1f);
	runFw();
	route();
	EXPECT_NEAR(_router->get_thrust_setpoint_0()->xyz[2], -.1f, 1e-6f);
}

TEST_F(TailsitterHandoffTest, CollectiveMatchingAndSlewUseBatteryScaledMotorCommands)
{
	batteryScale(2.f);
	begin(.37f);
	demand(.3f); // FW battery scaling makes this a .6 motor command.
	runFw();
	route();
	ASSERT_TRUE(active());
	expectHeld();
	runFw();
	route();
	expectHeld();

	for (int i = 0; i < 400; ++i) {
		demand(.3f);
		runFw();
		route();
	}

	ASSERT_FALSE(throttleLimited());
	EXPECT_NEAR(_router->get_thrust_setpoint_0()->xyz[2], -.6f, 1e-6f);
	demand(.1f);
	runFw();
	route();
	EXPECT_NEAR(_router->get_thrust_setpoint_0()->xyz[2], -.2f, 1e-6f);
}

TEST_F(TailsitterHandoffTest, AcroDirectYawRetainsTransferredBias)
{
	begin();
	directYaw();
	runFw();
	EXPECT_LT((bias() - Vector3f(_mc.torque_bias)).norm(), 1e-6f);
	const float transferred = directYawTrim();
	route();
	ASSERT_TRUE(active());
	expectHeld();
	runFw();
	route();
	expectHeld();

	for (int i = 0; i < 300; ++i) { runFw(); }

	EXPECT_FLOAT_EQ(directYawTrim(), transferred);
}

TEST_F(TailsitterHandoffTest, InvalidFwDemandHoldsLastValidMotorCommand)
{
	begin();
	demand(.5f);
	runFw();
	route();
	demand(NAN);
	runFw();
	route();
	expectHeld();
}

TEST_F(TailsitterHandoffTest, AbortThenRepeatReplacesBiasAndRejectsOldAck)
{
	for (float sign : {1.f, -1.f, 1.f}) {
		begin(.42f, sign);
		route();
		EXPECT_FALSE(active());
		expectHeld();
		demand(.3f);
		runFw();
		EXPECT_LT((bias() - Vector3f(_mc.torque_bias)).norm(), 1e-6f);
		route();
		ASSERT_TRUE(active());
		abort();
		EXPECT_EQ(initialized(), 0u);
		EXPECT_FALSE(throttleLimited());
	}
}

TEST_F(TailsitterHandoffTest, HoverRoutingIsUnchanged)
{
	Vector3f(.13f, -.27f, .31f).copyTo(_router->get_vehicle_torque_setpoint_virtual_mc()->xyz);
	_router->get_vehicle_thrust_setpoint_virtual_mc()->xyz[2] = -.44f;
	route();
	EXPECT_EQ(Vector3f(_router->get_torque_setpoint_0()->xyz), Vector3f(.13f, -.27f, .31f));
	EXPECT_FLOAT_EQ(_router->get_thrust_setpoint_0()->xyz[2], -.44f);
}

TEST_F(TailsitterHandoffTest, PostTecsAttitudeTargetMustArriveBeforeAcknowledgement)
{
	begin();
	attitudeSourceReadiness();
	EXPECT_TRUE(active());
	expectHeld();
}

TEST_F(TailsitterHandoffTest, ManualTargetCannotPairFreshTimestampWithOldThrust)
{
	begin();
	manualAttitudeSource();
	ASSERT_TRUE(active());
	expectHeld();
	runFw();
	route();
	expectHeld();
	runFw();
	route();
	EXPECT_NEAR(_router->get_thrust_setpoint_0()->xyz[2], -.37f, .002f);
}

TEST_F(TailsitterHandoffTest, PublicationLossHoldsThenQuadchutes)
{
	begin();
	demand(.7f);
	runFw();
	route();
	ASSERT_TRUE(active());
	expireOutputs();
	expectHeld();
	EXPECT_FALSE(_router->get_vtol_vehicle_status()->fixed_wing_system_failure);
	expireHandoff();
	expectHeld();
	EXPECT_TRUE(_router->get_vtol_vehicle_status()->fixed_wing_system_failure);
}

TEST_F(TailsitterHandoffTest, AbortBeforeAcknowledgementClearsTransfer)
{
	begin();
	demand(.7f, false);
	runFw();
	route();
	EXPECT_FALSE(active());
	abort();
	EXPECT_EQ(initialized(), 0u);
	begin(.25f, -1.f);
	demand(.7f);
	runFw();
	route();
	EXPECT_TRUE(active());
	expectHeld();
}

TEST_F(TailsitterHandoffTest, MotorSaturationUsesFixedWingAxisSigns)
{
	begin();
	demand(.7f);
	runFw();
	route();
	ASSERT_TRUE(active());
	saturatedAllocator();
}

TEST_F(TailsitterHandoffTest, SurfaceRollFeedforwardDoesNotStepDifferentialYaw)
{
	surfaceOnlyRoll();
	begin();
	demand(.7f);
	runFw();
	route();
	ASSERT_TRUE(active());
	runFw();
	route();
	// MC x is FW yaw; its feedforward must account for the independently controlled roll surface.
	EXPECT_NEAR(_router->get_torque_setpoint_0()->xyz[0], _mc.torque[0], 1e-6f);
	EXPECT_NEAR(_router->get_torque_setpoint_0()->xyz[1], _mc.torque[1], 1e-6f);
}
