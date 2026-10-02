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
#include "../mc_rate_control/MulticopterRateControl.hpp"

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
		float back_slew = 0.f;
		ASSERT_EQ(param_get(param_find("VT_TS_B_THR_SLEW"), &back_slew), 0);
		ASSERT_FLOAT_EQ(back_slew, .5f);
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
		control_allocator_status_s allocation{};
		allocation.timestamp = hrt_absolute_time();
		allocation.torque_setpoint_achieved = true;
		_allocation.publish(allocation);
		mode(true);
	}

	void mode(bool fw, bool transition = false)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.is_vtol = status.is_vtol_tailsitter = true;
		status.in_transition_mode = transition;
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
	std::unique_ptr<MulticopterRateControl> _mc_controller;
	uORB::Publication<vehicle_rates_setpoint_s> _mc_rates{ORB_ID(vehicle_rates_setpoint_virtual_mc)};
	uORB::Subscription _mc_torque{ORB_ID(vehicle_torque_setpoint_virtual_mc)};
	uORB::Subscription _mc_thrust{ORB_ID(vehicle_thrust_setpoint_virtual_mc)};
	Vector3f _back_torque{};
	Vector3f _back_bias{};
	float _back_thrust{0.f};

	void beginBack(float battery = 1.f, bool limited_integral = false)
	{
		begin();
		demand(.37f);
		runFw();
		route();
		runFw();
		route();
		_back_torque = Vector3f(_router->get_torque_setpoint_0()->xyz);
		_back_thrust = -_router->get_thrust_setpoint_0()->xyz[2];
		Quatf(matrix::Eulerf(0.f, math::radians(-80.f), 0.f)).copyTo(_tailsitter->_v_att->q);
		Quatf(matrix::Eulerf(0.f, math::radians(10.f), 0.f)).copyTo(_tailsitter->_fw_virtual_att_sp->q_d);
		Quatf(_tailsitter->_v_att->q).copyTo(_tailsitter->_v_att_sp->q_d);
		_tailsitter->startBackHandoff();
		ASSERT_TRUE(_tailsitter->_handoff.to_mc);
		ASSERT_NE(_tailsitter->_handoff.handoff_id, 0u);
		_back_bias = Vector3f(_tailsitter->_handoff.torque_bias);
		EXPECT_LT((_back_bias - Vector3f(_mc.torque_bias)).norm(), 1e-6f);
		_tailsitter->_vtol_mode = Tailsitter::vtol_mode::TRANSITION_BACK;
		_tailsitter->resetTransitionStates();
		mode(false, true);
		parameter("MC_BAT_SCALE_EN", int32_t(battery != 1.f));

		if (limited_integral) {
			for (const char *name : {"MC_RR_INT_LIM", "MC_PR_INT_LIM", "MC_YR_INT_LIM"}) { parameter(name, .02f); }
		}

		_mc_controller = std::make_unique<MulticopterRateControl>(true);
		battery_status_s status{};
		status.connected = true;
		status.scale = battery;
		_battery.publish(status);
		px4_usleep(1);
	}

	void mcDemand(float collective, bool fresh = true)
	{
		vehicle_rates_setpoint_s sp{};
		sp.timestamp = fresh ? hrt_absolute_time() : _tailsitter->_handoff.handoff_id - 1;
		sp.roll = -.3f;
		sp.pitch = .4f;
		sp.yaw = -.2f;
		sp.thrust_body[2] = -collective;
		_mc_rates.publish(sp);
	}

	void runMc()
	{
		vehicle_angular_velocity_s angular{};
		angular.timestamp = angular.timestamp_sample = hrt_absolute_time();
		Vector3f(.05f, -.07f, .09f).copyTo(angular.xyz);
		Vector3f(.2f, -.3f, .4f).copyTo(angular.xyz_derivative);
		_angular.publish(angular);
		_mc_controller->_last_run = angular.timestamp_sample - 10000; // 100 Hz sensor step.
		_mc_controller->Run();
	}

	void routeBack(bool thrust = true)
	{
		_mc_torque.copy(_router->get_vehicle_torque_setpoint_virtual_mc());

		if (thrust) { _mc_thrust.copy(_router->get_vehicle_thrust_setpoint_virtual_mc()); }

		_tailsitter->fill_actuator_outputs();
	}

	void expectBackHeld()
	{
		EXPECT_LT((Vector3f(_router->get_torque_setpoint_0()->xyz) - _back_torque).norm(), 1e-6f);
		EXPECT_NEAR(_router->get_thrust_setpoint_0()->xyz[2], -_back_thrust, 1e-6f);
	}

	void checkBackIntegral(float scale, float limit = 1.f)
	{
		EXPECT_LT((_mc_controller->_rate_control.getIntegral() - matrix::constrain(_back_bias / scale, -limit, limit)).norm(), 1e-6f);
	}

	void cancelBack()
	{
		_tailsitter->cancelHandoff();
		runMc();
		EXPECT_EQ(_mc_controller->_handoff_initialized, 0u);
		EXPECT_FALSE(_mc_controller->_throttle_slew.active());
	}

	void finishBack()
	{
		mode(false);
		_tailsitter->_vtol_mode = Tailsitter::vtol_mode::MC_MODE;
		_tailsitter->_back_transition_end_timestamp = hrt_absolute_time();
	}

	void delayedBackAttitudeSource()
	{
		_tailsitter->_flag_was_in_trans_mode = false;
		_tailsitter->_mc_virtual_att_sp->timestamp = 0;
		_tailsitter->_fw_virtual_att_sp->timestamp = hrt_absolute_time();
		_tailsitter->_last_thr_in_fw_mode = _back_thrust;
		Quatf(_tailsitter->_fw_virtual_att_sp->q_d).copyTo(_tailsitter->_v_att_sp->q_d);
		EXPECT_TRUE(_tailsitter->backHandoffSafe());
		_tailsitter->update_transition_state();
		EXPECT_TRUE(_tailsitter->_flag_was_in_trans_mode);
		EXPECT_LT(fabsf(matrix::Eulerf(Quatf(_tailsitter->_v_att_sp->q_d)).theta() - math::radians(-80.f)), .01f);
		EXPECT_FLOAT_EQ(_tailsitter->_v_att_sp->thrust_body[2], -_back_thrust);
		EXPECT_TRUE(_tailsitter->backHandoffSafe());
	}

	void backCompletionWaitsForController()
	{
		_tailsitter->_handoff_active_timestamp = hrt_absolute_time() - 600000;
		finishBack();
		_tailsitter->update_vtol_state(); // MC rate control has not observed this mode change yet.
		EXPECT_NE(_tailsitter->_handoff.handoff_id, 0u);
		mcDemand(.8f);
		runMc();
		_tailsitter->update_vtol_state();
		EXPECT_NE(_tailsitter->_handoff.handoff_id, 0u);

		for (int i = 0; i < 200 && backSlewActive(); ++i) { mcDemand(.8f); runMc(); routeBack(); }

		EXPECT_FALSE(backSlewActive());

		const Vector3f integral = mcIntegral();
		_tailsitter->_back_transition_end_timestamp = hrt_absolute_time() - 600000;
		_tailsitter->update_vtol_state();
		EXPECT_EQ(_tailsitter->_handoff.handoff_id, 0u);
		EXPECT_FALSE(_tailsitter->_handoff.to_mc);
		runMc();
		EXPECT_LT((mcIntegral() - integral).norm(), .01f);
	}

	void saturatedMcAllocator()
	{
		control_allocator_status_s allocation{};
		allocation.timestamp = hrt_absolute_time();
		Vector3f(.1f, -.1f, .1f).copyTo(allocation.unallocated_torque);
		_allocation.publish(allocation);
		const Vector3f integral = mcIntegral();
		runMc();
		EXPECT_EQ(mcIntegral(), integral);
	}

	void backAttitudeCompletion()
	{
		// Actual pitch crossed the completion threshold; the position controller now
		// asks for a braking attitude on the other side of vertical.
		Quatf(matrix::Eulerf(0.f, math::radians(-14.f), 0.f)).copyTo(_tailsitter->_v_att->q);
		_tailsitter->_q_trans_sp = Quatf(matrix::Eulerf(0.f, math::radians(-10.f), 0.f));
		_tailsitter->_q_trans_sp.copyTo(_tailsitter->_v_att_sp->q_d);
		Quatf(matrix::Eulerf(0.f, math::radians(17.f), 0.f)).copyTo(_tailsitter->_mc_virtual_att_sp->q_d);
		_tailsitter->update_vtol_state();
		_tailsitter->update_mc_state();
		EXPECT_NEAR(matrix::Eulerf(Quatf(_tailsitter->_v_att_sp->q_d)).theta(), math::radians(-10.f), .001f);
		_tailsitter->_back_transition_end_timestamp = hrt_absolute_time() - 250000;
		Quatf(matrix::Eulerf(0.f, math::radians(20.f), 0.f)).copyTo(_tailsitter->_mc_virtual_att_sp->q_d);
		_tailsitter->update_mc_state();
		EXPECT_NEAR(matrix::Eulerf(Quatf(_tailsitter->_v_att_sp->q_d)).theta(), math::radians(6.5f), .001f);
		_tailsitter->_back_transition_end_timestamp = hrt_absolute_time() - 600000;
		_tailsitter->update_mc_state();
		EXPECT_NEAR(matrix::Eulerf(Quatf(_tailsitter->_v_att_sp->q_d)).theta(), math::radians(20.f), .001f);
	}

	void expireBackHandoff()
	{
		_tailsitter->_handoff.handoff_id = hrt_absolute_time() - tailsitter_handoff::kOutputTimeout - 1;
		routeBack();
		EXPECT_EQ(_tailsitter->_handoff.handoff_id, 0u);
		EXPECT_TRUE(_tailsitter->_vtol_vehicle_status->fixed_wing_system_failure);
	}

	void commandedMcBraking()
	{
		_tailsitter->_vtol_mode = Tailsitter::vtol_mode::MC_MODE;
		tailsitter_handoff_s mc_state{};
		mc_state.rates[1] = math::radians(75.f);
		_mc_state.publish(mc_state);
		vehicle_angular_velocity_s angular{};
		angular.timestamp = hrt_absolute_time();
		angular.xyz[1] = math::radians(110.f);
		_angular.publish(angular);
		EXPECT_TRUE(_tailsitter->backHandoffSafe());
		angular.xyz[1] = math::radians(200.f);
		_angular.publish(angular);
		EXPECT_FALSE(_tailsitter->backHandoffSafe());
	}

	uint64_t backInitialized() const { return _mc_controller->_handoff_initialized; }
	Vector3f mcIntegral() const { return _mc_controller->_rate_control.getIntegral(); }
	bool backSlewActive() const { return _mc_controller->_throttle_slew.active(); }

	void rejectBack(int reason)
	{
		beginBack();
		_tailsitter->cancelHandoff();
		_tailsitter->_vtol_mode = Tailsitter::vtol_mode::FW_MODE;
		mode(true);

		if (reason == 0) {
			vehicle_angular_velocity_s angular{};
			angular.timestamp = hrt_absolute_time();
			angular.xyz[1] = math::radians(120.f);
			_angular.publish(angular);

		} else if (reason == 1) {
			_tailsitter->_vtol_vehicle_status->fixed_wing_system_failure = true;

		} else if (reason == 2) {
			control_allocator_status_s allocation{};
			allocation.timestamp = hrt_absolute_time();
			_allocation.publish(allocation);

		} else {
			tailsitter_handoff_s stale{};
			stale.timestamp_sample = 1;
			uORB::Publication<tailsitter_handoff_s> {ORB_ID(tailsitter_handoff_fw)}.publish(stale);
		}

		_tailsitter->startBackHandoff();
		EXPECT_EQ(_tailsitter->_handoff.handoff_id, 0u);
	}
	std::unique_ptr<VtolAttitudeControl> _router;
	std::unique_ptr<Tailsitter> _tailsitter;
	tailsitter_handoff_s _mc{};
	uORB::Publication<tailsitter_handoff_s> _mc_state{ORB_ID(tailsitter_handoff_mc)};
	uORB::Publication<vehicle_status_s> _vehicle{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_control_mode_s> _control{ORB_ID(vehicle_control_mode)};
	uORB::Publication<vehicle_land_detected_s> _land{ORB_ID(vehicle_land_detected)};
	uORB::Publication<control_allocator_status_s> _allocation{ORB_ID(control_allocator_status)};
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

TEST_F(TailsitterHandoffTest, BackWaitsForFreshMcDemandAndBothOutputs)
{
	beginBack();
	mcDemand(.8f, false);
	runMc();
	routeBack();
	EXPECT_EQ(backInitialized(), 0u);
	expectBackHeld();
	px4_usleep(60000);
	routeBack();
	expectBackHeld();
	mcDemand(.8f);
	runMc();
	routeBack(false);
	EXPECT_FALSE(active());
	expectBackHeld();
	routeBack();
	EXPECT_TRUE(active());
	expectBackHeld();
	checkBackIntegral(1.f);
	runMc();
	routeBack();
	expectBackHeld();
}

TEST_F(TailsitterHandoffTest, BackBiasAndCollectiveRespectBatteryScalingAndIntegralLimits)
{
	beginBack(2.f, true);
	mcDemand(.1f);
	runMc();
	checkBackIntegral(2.f, .02f);
	routeBack();
	ASSERT_TRUE(active());
	expectBackHeld();
	float trim = 0.f;
	param_get(param_find("TRIM_PITCH"), &trim);
	EXPECT_FLOAT_EQ(trim, -.02f);
}

TEST_F(TailsitterHandoffTest, BackSlewAcquiresDemandThenLeavesHoverResponseUnrestricted)
{
	beginBack();
	mcDemand(.1f);
	runMc();
	routeBack();
	ASSERT_TRUE(active());

	for (int i = 0; i < 80; ++i) {
		const float previous = _router->get_thrust_setpoint_0()->xyz[2];
		mcDemand(.1f);
		runMc();
		routeBack();
		EXPECT_LE(fabsf(_router->get_thrust_setpoint_0()->xyz[2] - previous), .01001f);
	}

	EXPECT_FALSE(backSlewActive());
	EXPECT_NEAR(_router->get_thrust_setpoint_0()->xyz[2], -.1f, 1e-6f);
	mcDemand(.9f);
	runMc();
	routeBack();
	EXPECT_NEAR(_router->get_thrust_setpoint_0()->xyz[2], -.9f, 1e-6f);
}

TEST_F(TailsitterHandoffTest, BackCompletionReacquiresCollectiveWithoutReseedingBias)
{
	beginBack();
	mcDemand(.37f);
	runMc();
	routeBack();

	for (int i = 0; i < 80; ++i) { mcDemand(.37f); runMc(); routeBack(); }

	const Vector3f integral = mcIntegral();
	finishBack();
	mcDemand(.8f);
	runMc();
	routeBack();
	EXPECT_NEAR(_router->get_thrust_setpoint_0()->xyz[2], -.37f, 1e-6f);
	EXPECT_LT((mcIntegral() - integral).norm(), .01f);
	ASSERT_TRUE(backSlewActive());
	mcDemand(.8f);
	runMc();
	routeBack();
	EXPECT_GT(_router->get_thrust_setpoint_0()->xyz[2], -.391f);
	EXPECT_LT(_router->get_thrust_setpoint_0()->xyz[2], -.37f);
}

TEST_F(TailsitterHandoffTest, BackRecoveryCancellationImmediatelyReleasesThrottle)
{
	beginBack();
	mcDemand(.9f);
	runMc();
	routeBack();
	ASSERT_TRUE(active());
	cancelBack();
	routeBack();
	EXPECT_NEAR(_router->get_thrust_setpoint_0()->xyz[2], -.9f, 1e-6f);
	EXPECT_LT(mcIntegral().norm(), .01f);
}

TEST_F(TailsitterHandoffTest, BackUpsetBypassesMatching) { rejectBack(0); }
TEST_F(TailsitterHandoffTest, BackQuadchuteBypassesMatching) { rejectBack(1); }
TEST_F(TailsitterHandoffTest, BackSaturationBypassesMatching) { rejectBack(2); }
TEST_F(TailsitterHandoffTest, BackStaleFwStateBypassesMatching) { rejectBack(3); }

TEST_F(TailsitterHandoffTest, BackInitialAttitudeDoesNotWaitForMcPositionControl)
{
	beginBack();
	delayedBackAttitudeSource();
}

TEST_F(TailsitterHandoffTest, BackNormalCompletionWaitsForMcModeAndRetainsIntegral)
{
	beginBack();
	mcDemand(.37f);
	runMc();
	routeBack();

	for (int i = 0; i < 60; ++i) { mcDemand(.37f); runMc(); routeBack(); }

	ASSERT_FALSE(backSlewActive());
	backCompletionWaitsForController();
}

TEST_F(TailsitterHandoffTest, BackImportedIntegralRespectsAllocatorAntiWindup)
{
	beginBack();
	mcDemand(.37f);
	runMc();
	routeBack();
	saturatedMcAllocator();
}

TEST_F(TailsitterHandoffTest, BackAttitudeCompletionRemovesOnlyInitialMismatch)
{
	beginBack();
	mcDemand(.37f);
	runMc();
	routeBack();
	backAttitudeCompletion();
}

TEST_F(TailsitterHandoffTest, BackMissingControllerTimesOutIntoRecovery)
{
	beginBack();
	expireBackHandoff();
}

TEST_F(TailsitterHandoffTest, BackRepeatedRequestCannotAcceptPreviousAcknowledgement)
{
	beginBack();
	mcDemand(.37f);
	runMc();
	routeBack();
	ASSERT_TRUE(active());
	cancelBack();
	beginBack();
	mcDemand(.8f, false);
	runMc();
	routeBack();
	EXPECT_FALSE(active());
	expectBackHeld();
	mcDemand(.8f);
	runMc();
	routeBack();
	EXPECT_TRUE(active());
	expectBackHeld();
}

TEST_F(TailsitterHandoffTest, BackCompletionAllowsCommandedBrakingButRejectsLossOfTracking)
{
	beginBack();
	commandedMcBraking();
}
