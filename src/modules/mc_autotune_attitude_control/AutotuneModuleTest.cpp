/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
#include <string>
#include <vector>
#include <lib/autotune/Progress.hpp>
#include <uORB/topics/log_message.h>
#include <uORB/topics/mavlink_log.h>
#include <hrt_work.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include "mc_autotune_attitude_control.hpp"
#include "../fw_autotune_attitude_control/fw_autotune_attitude_control.hpp"
#include "../mc_att_control/mc_att_control.hpp"
#include "../fw_att_control/FixedwingAttitudeControl.hpp"

// Drive the real modules synchronously. The functional test runner intentionally
// does not start the work queue: every input and Run() invocation is deterministic.
class AutotuneModuleTest : public ::testing::Test
{
protected:
	using Status = autotune_attitude_control_status_s;
	static constexpr uint8_t MC = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	static constexpr uint8_t FW = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	static void SetUpTestSuite()
	{
		hrt_init();
		hrt_work_queue_init();
		px4_log_initialize();
	}

	void SetUp() override
	{
		resetProgress();
		param_control_autosave(false);
		param_reset_all();
		int32_t aux = 0;
		param_set_no_notification(param_find("FW_AT_MAN_AUX"), &aux);
		// Reserve both VTOL torque instances before constructing their subscribers.
		vehicle_torque_setpoint_s torque{};
		torque.timestamp = hrt_absolute_time();
		ASSERT_TRUE(_mc_torque.publish(torque));
		ASSERT_TRUE(_fw_torque.publish(torque));
		ASSERT_EQ(_mc_torque.get_instance(), 0);
		ASSERT_EQ(_fw_torque.get_instance(), 1);
		_mc = std::make_unique<McAutotuneAttitudeControl>();
		_fw = std::make_unique<FwAutotuneAttitudeControl>(true);
		// Drain commands left in uORB by preceding cases.
		vehicle_command_s ignored{};

		while (_mc->_vehicle_command_sub.update(&ignored)) {}

		while (_fw->_vehicle_command_sub.update(&ignored)) {}

		manual_control_setpoint_s manual{};
		manual.timestamp = hrt_absolute_time();
		_manual.publish(manual);
	}

	void vehicle(uint8_t type, bool transition = false, bool vtol = true, bool tailsitter = false)
	{
		vehicle_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.vehicle_type = type;
		status.is_vtol = vtol;
		status.is_vtol_tailsitter = tailsitter;
		status.in_transition_mode = transition;
		status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		status.nav_state = vehicle_status_s::NAVIGATION_STATE_POSCTL;
		ASSERT_TRUE(_vehicle.publish(status));
	}

	void command(bool external = true)
	{
		vehicle_command_s command{};
		command.timestamp = hrt_absolute_time();
		command.command = vehicle_command_s::VEHICLE_CMD_DO_AUTOTUNE_ENABLE;
		command.param1 = 1.f;
		command.from_external = external;
		ASSERT_TRUE(_command.publish(command));
	}

	void run(bool fw_first = false)
	{
		vehicle_torque_setpoint_s torque{};
		torque.timestamp = hrt_absolute_time();
		vehicle_angular_velocity_s angular{};
		angular.timestamp = torque.timestamp;
		ASSERT_TRUE(_angular.publish(angular));
		ASSERT_TRUE(_mc_torque.publish(torque));
		ASSERT_TRUE(_fw_torque.publish(torque));

		if (fw_first) { runFw(); runMc(); } else { runMc(); runFw(); }
	}

	void runMc()
	{
		if (_mc) {
			_mc->_last_publish = 0;
			_mc->Run();
			_mc->ScheduleClear();
			_mc->_vehicle_torque_setpoint_sub.unregisterCallback();
		}
	}

	void runFw()
	{
		if (_fw) {
			_fw->_last_publish = 0;
			_fw->Run();
			_fw->ScheduleClear();
			_fw->_vehicle_torque_setpoint_sub.unregisterCallback();
		}
	}

	uint8_t mcState() { return static_cast<uint8_t>(_mc->_state); }
	uint8_t fwState() { return static_cast<uint8_t>(_fw->_state); }
	void expireMcTerminal() { _mc->_state_start_time = hrt_absolute_time() - 3_s; }
	void expireFwTerminal() { _fw->_state_start_time = hrt_absolute_time() - 3_s; }
	void advanceFwSine() { _fw->_state_start_time = hrt_absolute_time() - 250_ms; }
	void mcState(uint8_t state) { _mc->_state = static_cast<McAutotuneAttitudeControl::state>(state); }
	void fwState(uint8_t state) { _fw->_state = static_cast<FwAutotuneAttitudeControl::state>(state); }
	void aux(float value)
	{
		int32_t channel = 1;
		param_set_no_notification(param_find("FW_AT_MAN_AUX"), &channel);
		_fw->updateParams();
		manual_control_setpoint_s manual{};
		manual.timestamp = hrt_absolute_time();
		manual.aux1 = value;
		_manual.publish(manual);
	}

	void beginIdentification()
	{
		// Filter-rate collection is orthogonal to routing; start from its completed state.
		if (_mc) {
			ASSERT_TRUE(_mc->init());
			_mc->ScheduleClear();
			_mc->_are_filters_initialized = true;
		}

		if (_fw) {
			ASSERT_TRUE(_fw->init());
			_fw->ScheduleClear();
			_fw->_are_filters_initialized = true;
		}

		run();
		run();
	}

	void controllerInputs(bool tailsitter = false)
	{
		vehicle_attitude_s attitude{};
		attitude.timestamp = hrt_absolute_time();
		attitude.timestamp_sample = attitude.timestamp;
		const matrix::Quatf orientation = tailsitter ? matrix::Quatf(matrix::Eulerf(0.f, -M_PI_2_F, 0.f)) :
						  matrix::Quatf();
		orientation.copyTo(attitude.q);
		ASSERT_TRUE(_attitude.publish(attitude));
		vehicle_attitude_setpoint_s setpoint{};
		setpoint.timestamp = attitude.timestamp;
		setpoint.q_d[0] = 1.f;
		ASSERT_TRUE(_attitude_sp.publish(setpoint));
		vehicle_control_mode_s mode{};
		mode.timestamp = attitude.timestamp;
		mode.flag_control_attitude_enabled = true;
		mode.flag_control_rates_enabled = true;
		ASSERT_TRUE(_control_mode.publish(mode));
	}

	void runMcController(MulticopterAttitudeControl &controller)
	{
		controllerInputs();
		controller.Run();
	}

	void runFwController(FixedwingAttitudeControl &controller)
	{
		controllerInputs();
		controller.Run();
		controller.ScheduleClear();
	}

	bool ownerStatus(uint8_t type, const Status &status)
	{
		return type == MC ? _mc->_session.publish(status) : _fw->_session.publish(status);
	}

	void checkFwAxisRouting(bool vtol, bool tailsitter)
	{
		if (!vtol) {
			_fw = std::make_unique<FwAutotuneAttitudeControl>(false);
		}

		int32_t axes = 7;
		param_set_no_notification(param_find("FW_AT_AXES"), &axes);
		const float max_rate = 30.f;

		for (const char *name : {"FW_R_RMAX", "FW_P_RMAX_POS", "FW_P_RMAX_NEG", "FW_Y_RMAX"}) {
			param_set_no_notification(param_find(name), &max_rate);
		}

		_fw->updateParams();
		vehicle(FW, false, vtol, tailsitter);
		command();
		run();
		ASSERT_EQ(fwState(), Status::STATE_INIT);
		FixedwingAttitudeControl controller(vtol);
		uORB::Subscription rates{ORB_ID(vehicle_rates_setpoint)};
		auto controller_step = [&]() {
			controllerInputs(tailsitter);
			controller.Run();
			controller.ScheduleClear();
		};
		// Populate the controller's cached vehicle status before comparing rates.
		controller_step();
		controller_step();

		const uint8_t amplitude_states[] = {Status::STATE_ROLL_AMPLITUDE_DETECTION, Status::STATE_PITCH_AMPLITUDE_DETECTION, Status::STATE_YAW_AMPLITUDE_DETECTION};
		const uint8_t identification_states[] = {Status::STATE_ROLL, Status::STATE_PITCH, Status::STATE_YAW};

		for (int axis = 0; axis < 3; ++axis) {
			SCOPED_TRACE(axis);

			for (float sign : {1.f, -1.f}) {
				SCOPED_TRACE(sign);
				fwState(amplitude_states[axis]);
				_fw->_state_start_time = hrt_absolute_time();
				_fw->_signal_filter.reset(0.f);
				const matrix::Vector3f excitation = _fw->scaleInputSignal(sign * .5f);
				Status status{};
				status.state = amplitude_states[axis];
				excitation.copyTo(status.rate_sp);
				ASSERT_TRUE(ownerStatus(FW, status));
				controller_step();
				vehicle_rates_setpoint_s setpoint{};
				ASSERT_TRUE(rates.update(&setpoint));
				const matrix::Vector3f body_rates(setpoint.roll, setpoint.pitch, setpoint.yaw);
				const matrix::Vector3f expected_body = tailsitter ? matrix::Vector3f(excitation(2), excitation(1), -excitation(0)) : excitation;

				for (int i = 0; i < 3; ++i) {
					ASSERT_NEAR(body_rates(i), expected_body(i), 1e-5f);
				}

				// Use an ideal response to the real controller output to isolate frame
				// routing from aircraft dynamics. Torque uses the same body frame.
				vehicle_angular_velocity_s angular{};
				angular.timestamp = hrt_absolute_time();
				body_rates.copyTo(angular.xyz);
				ASSERT_TRUE(_angular.publish(angular));
				vehicle_torque_setpoint_s torque{};
				(body_rates * .2f).copyTo(torque.xyz);
				_fw->_are_filters_initialized = true;
				_fw->_input_scale = 2.f;
				_fw->_sample_interval_avg = .0025f;
				_fw->_sys_id.reset();
				_fw->_sys_id.setLpfCutoffFrequency(400.f, 30.f);
				_fw->_sys_id.setHpfCutoffFrequency(400.f, .5f);
				_fw->_sys_id.setFitnessLpfTimeConstant(1.f, .0025f);
				_fw->_sys_id.update(0.f, 0.f);
				_fw->_amplitude_detection_state = FwAutotuneAttitudeControl::amplitudeDetectionState::first_period;
				_fw->_time_last_amplitude_increase = hrt_absolute_time();
				_fw->_rate_reached = false;

				SystemIdentification expected;
				expected.setLpfCutoffFrequency(400.f, 30.f);
				expected.setHpfCutoffFrequency(400.f, .5f);
				expected.setFitnessLpfTimeConstant(1.f, .0025f);
				expected.update(0.f, 0.f);

				// Also enter identification with fresh torque but no new gyro sample:
				// the cached rate must not be rotated a second time.
				for (uint8_t state : {amplitude_states[axis], identification_states[axis]}) {
					fwState(state);
					torque.timestamp = hrt_absolute_time();
					ASSERT_TRUE(vtol ? _fw_torque.publish(torque) : _mc_torque.publish(torque));
					runFw();
					ASSERT_EQ(fwState(), state);
					expected.update(excitation(axis) * .2f * 2.f, excitation(axis));
					EXPECT_NEAR(_fw->_sys_id.getFilteredInputData(), expected.getFilteredInputData(), 1e-5f);
					EXPECT_NEAR(_fw->_sys_id.getFilteredOutputData(), expected.getFilteredOutputData(), 1e-5f);
					EXPECT_TRUE(_fw->_rate_reached);
				}
			}
		}
	}

	float gain(const char *name)
	{
		float value = NAN;
		EXPECT_EQ(param_get(param_find(name), &value), PX4_OK);
		return value;
	}

	void candidateGains(uint8_t type)
	{
		if (type == MC) {
			_mc->_rate_k = matrix::Vector3f(.08f, .08f, .08f);
			_mc->_rate_i = matrix::Vector3f(.3f, .3f, .3f);
			_mc->_rate_d = matrix::Vector3f(.002f, .002f, .002f);
			_mc->_att_p = matrix::Vector3f(4.f, 4.f, 3.f);

		} else {
			_fw->_rate_k = matrix::Vector3f(.08f, .08f, .08f);
			_fw->_rate_i = matrix::Vector3f(.3f, .3f, .3f);
			_fw->_rate_ff = matrix::Vector3f(.4f, .4f, .4f);
			_fw->_att_p = matrix::Vector3f(4.f, 4.f, 3.f);
		}
	}

	void trialGains(uint8_t type)
	{
		candidateGains(type);

		if (type == MC) {
			_mc->backupAndSaveGainsToParams();
			mcState(Status::STATE_TEST);

		} else {
			_fw->backupAndSaveGainsToParams();
			fwState(Status::STATE_TEST);
		}
	}

	void disarm()
	{
		uORB::Subscription vehicle_sub{ORB_ID(vehicle_status)};
		vehicle_status_s status{};
		ASSERT_TRUE(vehicle_sub.copy(&status));
		status.timestamp = hrt_absolute_time();
		status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
		ASSERT_TRUE(_vehicle.publish(status));
	}

	void loseInput(uint8_t type)
	{
		if (type == MC) { _mc->_last_control_input = hrt_absolute_time() - 2_s; }

		else { _fw->_last_control_input = hrt_absolute_time() - 2_s; }
	}


	void queueTestMessage() { _mc->_session.message("test record"); }

	void resetProgress()
	{
		auto &progress = autotune::Progress::instance();
		progress.ScheduleClear();
		progress._head = progress._count = progress._dropped = 0;
		progress._last_publish = 0;
		mavlink_log_s mavlink{};
		log_message_s log{};

		while (_mavlink_log.update(&mavlink)) {}

		while (_log_message.update(&log)) {}
	}

	std::vector<std::string> drainProgress()
	{
		std::vector<std::string> messages;
		auto &progress = autotune::Progress::instance();

		while (progress._count || progress._dropped) {
			progress.update(progress._last_publish + 100_ms);
			progress.ScheduleClear();
			mavlink_log_s mavlink{};
			EXPECT_TRUE(_mavlink_log.update(&mavlink));
			messages.emplace_back(mavlink.text);
			// Verify the same text and severity reach the ULog input, not just telemetry.
			log_message_s log{};
			bool found = false;

			while (_log_message.update(&log)) {
				if (strstr(log.text, mavlink.text)) {
					EXPECT_EQ(log.severity, mavlink.severity);
					found = true;
				}
			}

			EXPECT_TRUE(found) << mavlink.text;
			// Even with more queued records, an early call must not flood either sink.
			progress.update(progress._last_publish + 99_ms);
			progress.ScheduleClear();
			EXPECT_FALSE(_mavlink_log.updated());
			EXPECT_FALSE(_log_message.updated());
		}

		return messages;
	}

	void stepState(uint8_t type, uint8_t state, hrt_abstime elapsed = 0)
	{
		const auto now = hrt_absolute_time();

		if (type == MC) {
			mcState(state);
			_mc->_state_start_time = now - elapsed;
			_mc->updateStateMachine(now);

		} else {
			fwState(state);
			_fw->_state_start_time = now - elapsed;
			_fw->updateStateMachine(now);
		}
	}

	void fwAxes(int32_t axes)
	{
		param_set_no_notification(param_find("FW_AT_AXES"), &axes);
		_fw->updateParams();
	}

	void fwConvergedAxis(uint8_t state)
	{
		// Zero data makes the fitness metric settle; test only the transition/reporting.
		_fw->_sys_id.setFitnessLpfTimeConstant(.01f, .01f);

		for (int i = 0; i < 100; ++i) { _fw->_sys_id.update(0.f, 0.f); }

		stepState(FW, state, 6_s);
	}

	void stopTuner(uint8_t type)
	{

		if (type == MC) { _mc.reset(); } else { _fw.reset(); }
	}

	void setApply(uint8_t type, int32_t apply)
	{
		param_set_no_notification(param_find(type == MC ? "MC_AT_APPLY" : "FW_AT_APPLY"), &apply);

		if (type == MC) { _mc->updateParams(); } else { _fw->updateParams(); }
	}

	void checkParameterMessages(uint8_t type)
	{
		const std::vector<const char *> names = type == MC ? std::vector<const char *> {
			"MC_ROLLRATE_P", "MC_ROLLRATE_K", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_ROLL_P",
			"MC_PITCHRATE_P", "MC_PITCHRATE_K", "MC_PITCHRATE_I", "MC_PITCHRATE_D", "MC_PITCH_P",
			"MC_YAWRATE_P", "MC_YAWRATE_K", "MC_YAWRATE_I", "MC_YAWRATE_D", "MC_YAW_P"
} : std::vector<const char *> {
			"FW_RR_P", "FW_RR_I", "FW_RR_FF", "FW_R_TC", "FW_PR_P", "FW_PR_I", "FW_PR_FF", "FW_P_TC",
			"FW_YR_P", "FW_YR_I", "FW_YR_FF"
		};
		vehicle(type);
		command();
		run();
		fwAxes(7);
		drainProgress();
		std::vector<float> original;

		for (const char *name : names) { original.push_back(gain(name)); }

		trialGains(type);
		std::vector<float> trial;

		for (const char *name : names) { trial.push_back(gain(name)); }

		// Stop before any text is emitted: rollback and all pending records must survive.
		stopTuner(type);
		const auto messages = drainProgress();
		ASSERT_EQ(messages.size(), names.size() * 2 + 2);
		EXPECT_NE(messages[names.size()].find("FAIL: module stopped"), std::string::npos);
		EXPECT_NE(messages[names.size() + 1].find("restoring previous gains"), std::string::npos);

		for (unsigned i = 0; i < names.size(); ++i) {
			for (bool restoring : {false, true}) {
				const auto &message = messages[i + (restoring ? names.size() + 2 : 0)];
				const std::string prefix = std::string("AutoTune ") + (type == MC ? "MC: " : "FW: ") +
							   (restoring ? "restored " : "set ") + names[i] + "=";
				ASSERT_EQ(message.find(prefix), 0u) << message;
				EXPECT_FLOAT_EQ(std::stof(message.substr(prefix.size())), restoring ? gain(names[i]) : trial[i]);
			}
			EXPECT_NEAR(gain(names[i]), original[i], 1e-6f);
		}
	}

	uORB::Subscription _mavlink_log{ORB_ID(mavlink_log)};
	uORB::Subscription _log_message{ORB_ID(log_message)};

	std::unique_ptr<McAutotuneAttitudeControl> _mc;
	std::unique_ptr<FwAutotuneAttitudeControl> _fw;
	uORB::Publication<vehicle_status_s> _vehicle{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_command_s> _command{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_angular_velocity_s> _angular{ORB_ID(vehicle_angular_velocity)};
	uORB::Publication<manual_control_setpoint_s> _manual{ORB_ID(manual_control_setpoint)};
	uORB::PublicationMulti<vehicle_torque_setpoint_s> _mc_torque{ORB_ID(vehicle_torque_setpoint)};
	uORB::PublicationMulti<vehicle_torque_setpoint_s> _fw_torque{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_attitude_s> _attitude{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_sp{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_control_mode_s> _control_mode{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _status{ORB_ID(autotune_attitude_control_status)};
};

TEST_F(AutotuneModuleTest, HoverExternalCommandStartsOnlyMc)
{
	vehicle(MC);
	command();
	run();
	EXPECT_EQ(mcState(), Status::STATE_INIT);
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.state, Status::STATE_INIT);
	EXPECT_EQ(status.vehicle_type, MC);
	EXPECT_NE(status.timestamp_start, 0);
}

TEST_F(AutotuneModuleTest, HoverMissionCommandStartsOnlyMcInEitherRunOrder)
{
	vehicle(MC);
	command(false);
	run(true);
	EXPECT_EQ(mcState(), Status::STATE_INIT);
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
}

TEST_F(AutotuneModuleTest, FixedWingCommandStartsOnlyFw)
{
	vehicle(FW);
	command();
	run();
	EXPECT_EQ(mcState(), Status::STATE_IDLE);
	EXPECT_EQ(fwState(), Status::STATE_INIT);
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.vehicle_type, FW);
}

TEST_F(AutotuneModuleTest, TransitionRejectsCommandWithoutDeferredStart)
{
	vehicle(MC, true);
	command();
	run();
	EXPECT_EQ(mcState(), Status::STATE_IDLE);
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
	vehicle(FW);
	run();
	EXPECT_EQ(mcState(), Status::STATE_IDLE);
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
}

TEST_F(AutotuneModuleTest, UnavailableVehicleStatusRejectsCommand)
{
	vehicle_status_s unavailable{};
	ASSERT_TRUE(_vehicle.publish(unavailable));
	command();
	run();
	EXPECT_EQ(mcState(), Status::STATE_IDLE);
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
	vehicle(MC);
	run();
	EXPECT_EQ(mcState(), Status::STATE_IDLE);
}

TEST_F(AutotuneModuleTest, PollingKeepsTheSameAttempt)
{
	vehicle(MC);
	command();
	run();
	Status first{};
	ASSERT_TRUE(_status.copy(&first));

	for (int i = 0; i < 5; ++i) {
		command();
		run();
		Status status{};
		ASSERT_TRUE(_status.copy(&status));
		EXPECT_EQ(status.timestamp_start, first.timestamp_start);
		EXPECT_EQ(fwState(), Status::STATE_IDLE);
	}
}

TEST_F(AutotuneModuleTest, RegimeChangeAbortsMcAndDoesNotStartFw)
{
	vehicle(MC);
	command();
	run();
	vehicle(FW);
	run(true);
	EXPECT_EQ(mcState(), Status::STATE_FAIL);
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.state, Status::STATE_FAIL);

	for (float rate : status.rate_sp) { EXPECT_FLOAT_EQ(rate, 0.f); }

	command();
	run(true);
	EXPECT_EQ(fwState(), Status::STATE_INIT);
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.vehicle_type, FW);
	EXPECT_EQ(status.state, Status::STATE_INIT);
}

TEST_F(AutotuneModuleTest, TransitionAbortsFwPendingApplication)
{
	vehicle(FW);
	command();
	run();
	candidateGains(FW);
	const float original = gain("FW_RR_P");
	fwState(Status::STATE_WAIT_FOR_DISARM);
	vehicle(FW, true);
	disarm();
	run();
	EXPECT_EQ(fwState(), Status::STATE_FAIL);
	EXPECT_FLOAT_EQ(gain("FW_RR_P"), original);
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.state, Status::STATE_FAIL);
}

TEST_F(AutotuneModuleTest, TransitionAbortsMcPendingApplication)
{
	vehicle(MC);
	command();
	run();
	candidateGains(MC);
	const float original = gain("MC_ROLLRATE_P");
	mcState(Status::STATE_WAIT_FOR_DISARM);
	vehicle(MC, true);
	disarm();
	run();
	EXPECT_EQ(mcState(), Status::STATE_FAIL);
	EXPECT_FLOAT_EQ(gain("MC_ROLLRATE_P"), original);
}

TEST_F(AutotuneModuleTest, RejectedAuxMustBeReleasedBeforeFixedWingStart)
{
	vehicle(MC);
	aux(1.f);
	run();
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
	vehicle(FW);
	run();
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
	aux(0.f);
	run();
	aux(1.f);
	run();
	EXPECT_EQ(fwState(), Status::STATE_INIT);
}

TEST_F(AutotuneModuleTest, StoppingInactiveFwPreservesMcPublicationAndRestart)
{
	vehicle(MC);
	command();
	run();
	_fw.reset();
	run();
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.vehicle_type, MC);
	_mc.reset();
	_mc = std::make_unique<McAutotuneAttitudeControl>();
	vehicle(MC);
	command();
	run();
	EXPECT_EQ(mcState(), Status::STATE_INIT);
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.vehicle_type, MC);
}

TEST_F(AutotuneModuleTest, StoppingInactiveMcPreservesFwPublicationAndRestart)
{
	vehicle(FW);
	command();
	run();
	_mc.reset();
	run();
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.vehicle_type, FW);
	_fw.reset();
	_fw = std::make_unique<FwAutotuneAttitudeControl>(true);
	vehicle(FW);
	command();
	run();
	EXPECT_EQ(fwState(), Status::STATE_INIT);
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.vehicle_type, FW);
}

TEST_F(AutotuneModuleTest, SecondMcAttemptAfterAbortNeedsANewCommand)
{
	vehicle(MC);
	command();
	run();
	Status first{};
	ASSERT_TRUE(_status.copy(&first));
	vehicle(MC, true);
	run();
	expireMcTerminal();
	vehicle(MC);
	run();
	EXPECT_EQ(mcState(), Status::STATE_IDLE);
	run();
	EXPECT_EQ(mcState(), Status::STATE_IDLE);
	command();
	run();
	EXPECT_EQ(mcState(), Status::STATE_INIT);
	Status second{};
	ASSERT_TRUE(_status.copy(&second));
	EXPECT_GT(second.timestamp_start, first.timestamp_start);
}

TEST_F(AutotuneModuleTest, StandaloneMcAcceptsCommand)
{
	_fw.reset();
	vehicle(MC, false, false);
	command();
	run();
	EXPECT_EQ(mcState(), Status::STATE_INIT);
}

TEST_F(AutotuneModuleTest, StandaloneFwAcceptsCommand)
{
	_mc.reset();
	_fw.reset();
	_fw = std::make_unique<FwAutotuneAttitudeControl>(false);
	vehicle(FW, false, false);
	command();
	run();
	EXPECT_EQ(fwState(), Status::STATE_INIT);
}

TEST_F(AutotuneModuleTest, McExcitationReachesControllerAfterInactiveFwStops)
{
	vehicle(MC);
	command();
	run();
	beginIdentification();
	_fw.reset();
	run();
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	ASSERT_EQ(status.state, Status::STATE_ROLL);
	ASSERT_GT(fabsf(status.rate_sp[0]), 0.01f);
	MulticopterAttitudeControl controller(true);
	uORB::Subscription rates{ORB_ID(vehicle_rates_setpoint)};
	runMcController(controller);
	vehicle_rates_setpoint_s setpoint{};
	ASSERT_TRUE(rates.update(&setpoint));
	EXPECT_NEAR(setpoint.roll, status.rate_sp[0], 1e-5f);
	EXPECT_NEAR(setpoint.pitch, status.rate_sp[1], 1e-5f);
	EXPECT_NEAR(setpoint.yaw, status.rate_sp[2], 1e-5f);
}

TEST_F(AutotuneModuleTest, FwExcitationReachesControllerAfterInactiveMcStops)
{
	vehicle(FW);
	command();
	run();
	beginIdentification();
	_mc.reset();
	advanceFwSine();
	run();
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	ASSERT_EQ(status.state, Status::STATE_ROLL_AMPLITUDE_DETECTION);
	ASSERT_GT(fabsf(status.rate_sp[0]), 0.01f);
	FixedwingAttitudeControl controller(true);
	uORB::Subscription rates{ORB_ID(vehicle_rates_setpoint)};
	runFwController(controller);
	vehicle_rates_setpoint_s setpoint{};
	ASSERT_TRUE(rates.update(&setpoint));
	EXPECT_NEAR(setpoint.roll, status.rate_sp[0], 1e-5f);
	EXPECT_NEAR(setpoint.pitch, status.rate_sp[1], 1e-5f);
	EXPECT_NEAR(setpoint.yaw, status.rate_sp[2], 1e-5f);
}

TEST_F(AutotuneModuleTest, FwTailsitterIdentificationAndAmplitudeUseExcitedAxes)
{
	checkFwAxisRouting(true, true);
}

TEST_F(AutotuneModuleTest, FwStandardVtolIdentificationAndAmplitudeUseExcitedAxes)
{
	checkFwAxisRouting(true, false);
}

TEST_F(AutotuneModuleTest, FwStandaloneIdentificationAndAmplitudeUseExcitedAxes)
{
	checkFwAxisRouting(false, false);
}

TEST_F(AutotuneModuleTest, McControllerRejectsPreviousFwExcitation)
{
	vehicle(FW);
	command();
	run();
	Status status{};
	status.timestamp = hrt_absolute_time();
	status.state = Status::STATE_ROLL;
	status.rate_sp[0] = 0.25f;
	ASSERT_TRUE(ownerStatus(FW, status));
	vehicle(MC);
	MulticopterAttitudeControl controller(true);
	uORB::Subscription rates{ORB_ID(vehicle_rates_setpoint)};
	runMcController(controller);
	vehicle_rates_setpoint_s setpoint{};
	ASSERT_TRUE(rates.update(&setpoint));
	EXPECT_FLOAT_EQ(setpoint.roll, 0.f);
}

TEST_F(AutotuneModuleTest, FwControllerRejectsPreviousMcExcitationAndTransition)
{
	vehicle(MC);
	command();
	run();
	Status status{};
	status.timestamp = hrt_absolute_time();
	status.state = Status::STATE_ROLL;
	status.rate_sp[0] = 0.25f;
	ASSERT_TRUE(ownerStatus(MC, status));
	vehicle(FW);
	FixedwingAttitudeControl controller(true);
	uORB::Subscription rates{ORB_ID(vehicle_rates_setpoint)};
	runFwController(controller);
	vehicle_rates_setpoint_s setpoint{};
	ASSERT_TRUE(rates.update(&setpoint));
	EXPECT_FLOAT_EQ(setpoint.roll, 0.f);
	// Let MC abort, then explicitly start FW and retain a live FW excitation
	// sample while the transition flag arrives before the tuner runs again.
	run();
	command();
	run();
	ASSERT_TRUE(ownerStatus(FW, status));
	vehicle(FW, true);
	runFwController(controller);
	ASSERT_TRUE(rates.update(&setpoint));
	EXPECT_FLOAT_EQ(setpoint.roll, 0.f);
}

TEST_F(AutotuneModuleTest, DelayedWrongTunerCannotReuseAcceptedCommandAfterRegimeChange)
{
	vehicle(MC);
	command();
	runMc();
	ASSERT_EQ(mcState(), Status::STATE_INIT);
	vehicle(FW);
	runMc();
	ASSERT_EQ(mcState(), Status::STATE_FAIL);
	// FW first consumes the same, still fresh uORB command after MC released ownership.
	runFw();
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
}

TEST_F(AutotuneModuleTest, McInputLossAbortsTestAndRestoresGains)
{
	vehicle(MC);
	command();
	run();
	const float original = gain("MC_ROLLRATE_P");
	trialGains(MC);
	EXPECT_NE(gain("MC_ROLLRATE_P"), original);
	loseInput(MC);
	runMc();
	EXPECT_EQ(mcState(), Status::STATE_FAIL);
	EXPECT_NEAR(gain("MC_ROLLRATE_P"), original, 1e-6f);
}

TEST_F(AutotuneModuleTest, FwInputLossAbortsTestAndRestoresGains)
{
	vehicle(FW);
	command();
	run();
	const float original = gain("FW_RR_P");
	trialGains(FW);
	EXPECT_NE(gain("FW_RR_P"), original);
	loseInput(FW);
	runFw();
	EXPECT_EQ(fwState(), Status::STATE_FAIL);
	EXPECT_NEAR(gain("FW_RR_P"), original, 1e-6f);
}

TEST_F(AutotuneModuleTest, StopActiveFwRestoresTrialGainsAndPublishesZero)
{
	vehicle(FW);
	command();
	run();
	const float original = gain("FW_RR_P");
	trialGains(FW);
	_fw.reset();
	EXPECT_NEAR(gain("FW_RR_P"), original, 1e-6f);
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.state, Status::STATE_FAIL);

	for (float rate : status.rate_sp) { EXPECT_FLOAT_EQ(rate, 0.f); }
}

TEST_F(AutotuneModuleTest, StopActiveMcRestoresTrialGainsAndPublishesZero)
{
	vehicle(MC);
	command();
	run();
	const float original = gain("MC_ROLLRATE_P");
	trialGains(MC);
	_mc.reset();
	EXPECT_NEAR(gain("MC_ROLLRATE_P"), original, 1e-6f);
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.state, Status::STATE_FAIL);

	for (float rate : status.rate_sp) { EXPECT_FLOAT_EQ(rate, 0.f); }
}

TEST_F(AutotuneModuleTest, SecondFwAttemptAfterAbortNeedsANewCommand)
{
	vehicle(FW);
	command();
	run();
	Status first{};
	ASSERT_TRUE(_status.copy(&first));
	vehicle(FW, true);
	run();
	expireFwTerminal();
	vehicle(FW);
	run();
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
	run();
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
	command();
	run();
	EXPECT_EQ(fwState(), Status::STATE_INIT);
	Status second{};
	ASSERT_TRUE(_status.copy(&second));
	EXPECT_GT(second.timestamp_start, first.timestamp_start);
}

TEST_F(AutotuneModuleTest, StopInactiveFwBeforeFirstRequest)
{
	_fw.reset();
	vehicle(MC);
	command();
	run();
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.vehicle_type, MC);
	EXPECT_EQ(status.state, Status::STATE_INIT);
}

TEST_F(AutotuneModuleTest, StopInactiveMcBeforeFirstRequest)
{
	_mc.reset();
	vehicle(FW);
	command();
	run();
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.vehicle_type, FW);
	EXPECT_EQ(status.state, Status::STATE_INIT);
}

TEST_F(AutotuneModuleTest, AuxOnDuringTransitionMustBeReleasedBeforeStart)
{
	vehicle(FW, true);
	aux(1.f);
	run();
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
	vehicle(FW);
	run();
	EXPECT_EQ(fwState(), Status::STATE_IDLE);
	aux(0.f);
	run();
	aux(1.f);
	run();
	EXPECT_EQ(fwState(), Status::STATE_INIT);
}

TEST_F(AutotuneModuleTest, McPendingApplicationHasHeartbeatAndAppliesOnEligibleDisarm)
{
	vehicle(MC);
	command();
	run();
	candidateGains(MC);
	mcState(Status::STATE_WAIT_FOR_DISARM);
	loseInput(MC);
	runMc();
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.state, Status::STATE_WAIT_FOR_DISARM);
	EXPECT_NE(gain("MC_ROLLRATE_P"), .08f);
	disarm();
	runMc();
	EXPECT_EQ(mcState(), Status::STATE_COMPLETE);
	EXPECT_FLOAT_EQ(gain("MC_ROLLRATE_P"), .08f);
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.state, Status::STATE_COMPLETE);
}

TEST_F(AutotuneModuleTest, FwPendingApplicationHasHeartbeatAndAppliesOnEligibleDisarm)
{
	vehicle(FW);
	command();
	run();
	candidateGains(FW);
	fwState(Status::STATE_WAIT_FOR_DISARM);
	loseInput(FW);
	runFw();
	Status status{};
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.state, Status::STATE_WAIT_FOR_DISARM);
	EXPECT_NE(gain("FW_RR_P"), .08f);
	disarm();
	runFw();
	EXPECT_EQ(fwState(), Status::STATE_COMPLETE);
	EXPECT_FLOAT_EQ(gain("FW_RR_P"), .08f);
	ASSERT_TRUE(_status.copy(&status));
	EXPECT_EQ(status.state, Status::STATE_COMPLETE);
}

TEST_F(AutotuneModuleTest, RegimeChangeDuringMcTestRestoresGains)
{
	vehicle(MC);
	command();
	run();
	const float original = gain("MC_ROLLRATE_P");
	trialGains(MC);
	vehicle(FW);
	run();
	EXPECT_EQ(mcState(), Status::STATE_FAIL);
	EXPECT_NEAR(gain("MC_ROLLRATE_P"), original, 1e-6f);
	run();
	EXPECT_NEAR(gain("MC_ROLLRATE_P"), original, 1e-6f);
}

TEST_F(AutotuneModuleTest, RegimeChangeDuringFwTestRestoresGains)
{
	vehicle(FW);
	command();
	run();
	const float original = gain("FW_RR_P");
	trialGains(FW);
	vehicle(MC);
	run();
	EXPECT_EQ(fwState(), Status::STATE_FAIL);
	EXPECT_NEAR(gain("FW_RR_P"), original, 1e-6f);
	run();
	EXPECT_NEAR(gain("FW_RR_P"), original, 1e-6f);
}


TEST_F(AutotuneModuleTest, McProgressNamesEachAxisOnce)
{
	vehicle(MC);
	command();
	beginIdentification();
	EXPECT_EQ(drainProgress(), (std::vector<std::string> {"AutoTune MC: started; initializing", "AutoTune MC: roll tuning"}));
	run();
	EXPECT_TRUE(drainProgress().empty());
	stepState(MC, Status::STATE_ROLL_PAUSE, 3_s);
	stepState(MC, Status::STATE_PITCH_PAUSE, 3_s);
	EXPECT_EQ(drainProgress(), (std::vector<std::string> {"AutoTune MC: pitch tuning", "AutoTune MC: yaw tuning"}));
	loseInput(MC);
	runMc();
	EXPECT_EQ(drainProgress(), (std::vector<std::string> {"AutoTune MC: yaw FAIL: controller input lost", "AutoTune MC: FAIL: controller input lost"}));
}

TEST_F(AutotuneModuleTest, FwProgressReportsIdentificationPassAndDisabledAxis)
{
	vehicle(FW);
	command();
	beginIdentification();
	EXPECT_EQ(drainProgress(), (std::vector<std::string> {"AutoTune FW: started; initializing", "AutoTune FW: roll finding excitation amplitude"}));
	fwConvergedAxis(Status::STATE_ROLL);
	EXPECT_EQ(drainProgress(), (std::vector<std::string> {"AutoTune FW: roll PASS: identification"}));
	stepState(FW, Status::STATE_ROLL_PAUSE, 3_s);
	fwConvergedAxis(Status::STATE_PITCH);
	stepState(FW, Status::STATE_PITCH_PAUSE, 3_s);
	stepState(FW, Status::STATE_YAW_AMPLITUDE_DETECTION);
	EXPECT_EQ(drainProgress(), (std::vector<std::string> {"AutoTune FW: pitch finding excitation amplitude", "AutoTune FW: pitch PASS: identification", "AutoTune FW: yaw SKIPPED: disabled"}));
}

TEST_F(AutotuneModuleTest, FwProgressReportsAmplitudeTimeoutAsFailure)
{
	vehicle(FW);
	command();
	beginIdentification();
	drainProgress();
	stepState(FW, Status::STATE_ROLL_AMPLITUDE_DETECTION, 31_s);
	EXPECT_EQ(drainProgress(), (std::vector<std::string> {"AutoTune FW: roll FAIL: timeout; skipping"}));
	EXPECT_EQ(fwState(), Status::STATE_ROLL_PAUSE);
}

TEST_F(AutotuneModuleTest, McProgressAuditsEveryWriteAndRollbackAfterStop)
{
	checkParameterMessages(MC);
}

TEST_F(AutotuneModuleTest, FwProgressAuditsEveryWriteAndRollbackAfterStop)
{
	checkParameterMessages(FW);
}

TEST_F(AutotuneModuleTest, ProgressDoesNotClaimWritesWhenApplicationDisabled)
{
	for (uint8_t type : {MC, FW}) {
		vehicle(type);
		command();
		run();
		drainProgress();
		setApply(type, 0);
		stepState(type, Status::STATE_APPLY);
		EXPECT_EQ(drainProgress(), (std::vector<std::string> {std::string("AutoTune ") + (type == MC ? "MC" : "FW") + ": complete; gains not applied"}));
		stopTuner(type);
	}
}

TEST_F(AutotuneModuleTest, ProgressPacesBurstsAndExplicitlyReportsOverflow)
{
	// Exercise the real queue without starting a tuner or writing a parameter.
	for (unsigned i = 0; i < 67; ++i) { queueTestMessage(); }

	const auto messages = drainProgress();
	ASSERT_EQ(messages.size(), 65u);
	EXPECT_EQ(messages.back(), "AutoTune: progress queue overflow (3 lost)");
}
