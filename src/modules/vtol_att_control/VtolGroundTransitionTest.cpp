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

class FrontCompletionProbe : public Tailsitter
{
public:
	explicit FrontCompletionProbe(VtolAttitudeControl *controller) : Tailsitter(controller) {}
	using VtolType::isFrontTransitionCompleted;
	bool normal_completion{false};
protected:
	bool isFrontTransitionCompletedBase() override { return normal_completion; }
};

class VtolGroundTransitionTest : public ::testing::Test
{
protected:
	static void SetUpTestSuite()
	{
		hrt_init();
		hrt_work_queue_init();
		px4_log_initialize();
	}

	void SetUp() override
	{
		param_control_autosave(false);
		const int32_t type = 0;
		ASSERT_EQ(param_set_no_notification(param_find("VT_TYPE"), &type), 0);
		controller = std::make_unique<VtolAttitudeControl>();
		state(true, true);
	}

	void state(bool armed, bool landed)
	{
		controller->_vehicle_control_mode.flag_armed = armed;
		controller->_land_detected.landed = landed;
		controller->_vehicle_status.nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		_control.publish(controller->_vehicle_control_mode);
		_land.publish(controller->_land_detected);
		_vehicle.publish(controller->_vehicle_status);
	}

	void command(bool fw, bool external = false)
	{
		vehicle_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.command = vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION;
		cmd.param1 = fw ? vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW : vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
		cmd.from_external = external;
		_command.publish(cmd);
		controller->vehicle_cmd_poll();
	}

	void rc(bool fw)
	{
		action_request_s request{};
		request.timestamp = hrt_absolute_time();
		request.action = fw ? action_request_s::ACTION_VTOL_TRANSITION_TO_FIXEDWING : action_request_s::ACTION_VTOL_TRANSITION_TO_MULTICOPTER;
		_action.publish(request);
		controller->action_request_poll();
	}

	void tick() { controller->_vtol_type->update_vtol_state(); }
	::mode currentMode() { return controller->_vtol_type->get_mode(); }

	void run()
	{
		vehicle_torque_setpoint_s torque{};
		torque.timestamp = hrt_absolute_time();
		_torque.publish(torque);
		px4_usleep(3000);
		controller->Run();
		controller->ScheduleClear();
	}

	std::unique_ptr<VtolAttitudeControl> controller;
	uORB::Publication<vehicle_command_s> _command{ORB_ID(vehicle_command)};
	uORB::Publication<action_request_s> _action{ORB_ID(action_request)};
	uORB::Publication<vehicle_control_mode_s> _control{ORB_ID(vehicle_control_mode)};
	uORB::Publication<vehicle_land_detected_s> _land{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_status_s> _vehicle{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque{ORB_ID(vehicle_torque_setpoint_virtual_mc)};
};

TEST_F(VtolGroundTransitionTest, MissionTransitionIsRejectedOnGroundAndNotDeferred)
{
	command(true);
	tick();
	EXPECT_FALSE(controller->is_fixed_wing_requested());
	EXPECT_EQ(currentMode(), ::mode::ROTARY_WING);
	state(true, false);
	tick();
	EXPECT_EQ(currentMode(), ::mode::ROTARY_WING);
}

TEST_F(VtolGroundTransitionTest, ExternalGroundRequestGetsRejectedAcknowledgement)
{
	uORB::Subscription ack_sub{ORB_ID(vehicle_command_ack)};
	command(true, true);
	vehicle_command_ack_s ack{};
	ASSERT_TRUE(ack_sub.update(&ack));
	EXPECT_EQ(ack.result, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
}

TEST_F(VtolGroundTransitionTest, RcGroundRequestIsDiscarded)
{
	rc(true);
	tick();
	EXPECT_FALSE(controller->is_fixed_wing_requested());
	state(true, false);
	tick();
	EXPECT_EQ(currentMode(), ::mode::ROTARY_WING);
}

TEST_F(VtolGroundTransitionTest, DisarmedGroundChecksStillWork)
{
	state(false, true);
	command(true);
	tick();
	tick();
	EXPECT_EQ(currentMode(), ::mode::FIXED_WING);
}

TEST_F(VtolGroundTransitionTest, AirborneRequestStartsNormalTransition)
{
	state(true, false);
	command(true);
	tick();
	EXPECT_EQ(currentMode(), ::mode::TRANSITION_TO_FW);
}

TEST_F(VtolGroundTransitionTest, GroundReturnToMulticopterIsAllowed)
{
	state(false, true);
	command(true);
	tick();
	tick();
	state(true, true);
	command(false);
	EXPECT_FALSE(controller->is_fixed_wing_requested());
}

TEST_F(VtolGroundTransitionTest, RequestLatchedBeforeArmingIsDiscarded)
{
	state(false, true);
	command(true);
	ASSERT_TRUE(controller->is_fixed_wing_requested());
	state(true, true);
	run();
	EXPECT_FALSE(controller->is_fixed_wing_requested());
	EXPECT_EQ(currentMode(), ::mode::ROTARY_WING);
	state(true, false);
	run();
	EXPECT_EQ(currentMode(), ::mode::ROTARY_WING);
}

TEST_F(VtolGroundTransitionTest, LandedNeverCompletesArmedFrontTransition)
{
	FrontCompletionProbe probe(controller.get());
	probe.normal_completion = true;
	EXPECT_FALSE(probe.isFrontTransitionCompleted());
	state(true, false);
	EXPECT_TRUE(probe.isFrontTransitionCompleted());
	probe.normal_completion = false;
	EXPECT_FALSE(probe.isFrontTransitionCompleted());
	state(false, true);
	EXPECT_TRUE(probe.isFrontTransitionCompleted());
}
