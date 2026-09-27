/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

/**
 * @file mc_autotune_attitude_control.cpp
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#include "mc_autotune_attitude_control.hpp"

using namespace matrix;

ModuleBase::Descriptor McAutotuneAttitudeControl::desc{task_spawn, custom_command, print_usage};

McAutotuneAttitudeControl::McAutotuneAttitudeControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

McAutotuneAttitudeControl::~McAutotuneAttitudeControl()
{
	if (_state != state::idle && _state != state::complete && _state != state::fail) {
		abortAutotune(hrt_absolute_time());
	}

	_session.release();
	perf_free(_cycle_perf);
}

bool McAutotuneAttitudeControl::init()
{

	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_signal_filter.setParameters(_publishing_dt_s, .2f); // runs in the slow publishing loop

	ScheduleDelayed(_publishing_dt_hrt);
	return true;
}

void McAutotuneAttitudeControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_parameter_update_sub.unregisterCallback();
		_vehicle_torque_setpoint_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	// Keep checking flight-regime changes and timeouts even if the torque stream stops.
	ScheduleDelayed(_publishing_dt_hrt);
	updateVehicleStatus();
	const hrt_abstime now = hrt_absolute_time();

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	if (_actuator_controls_status_sub.updated()) {
		actuator_controls_status_s controls_status;

		if (_actuator_controls_status_sub.copy(&controls_status)) {
			_control_power = Vector3f(controls_status.control_power);
		}
	}

	_vehicle_cmd_start_autotune = false;
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		const hrt_abstime command_now = hrt_absolute_time();

		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_AUTOTUNE_ENABLE
		    && fabsf(vehicle_command.param1 - 1.0f) < FLT_EPSILON && fabsf(vehicle_command.param2) < FLT_EPSILON
		    && _state == state::idle
		    && vehicle_command.timestamp != 0 && command_now >= vehicle_command.timestamp
		    && command_now - vehicle_command.timestamp <= 2_s
		    && autotune::eligible(_vehicle_status, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, now)) {
			_vehicle_cmd_start_autotune = true;
			_start_request_timestamp = vehicle_command.timestamp;
		}
	}

	checkAbort(now);

	if (_state == state::complete || _state == state::fail || _state == state::wait_for_disarm) {
		updateStateMachine(now);
		publishState(now);

		if (_state == state::idle) {
			_session.release();
		}

		return;
	}

	if (_state == state::idle && _vehicle_cmd_start_autotune) {
		updateStateMachine(now);
		publishState(now);
	}

	// New control data is needed for identification, but never for abort/cleanup.
	if (_state == state::idle || !_vehicle_torque_setpoint_sub.updated()) {
		return;
	}

	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_angular_velocity_s angular_velocity;

	if (!_vehicle_torque_setpoint_sub.copy(&vehicle_torque_setpoint)
	    || !_vehicle_angular_velocity_sub.copy(&angular_velocity)) {
		return;
	}

	// Do not keep exciting a plant whose controller or gyro stream has stopped.
	if (vehicle_torque_setpoint.timestamp == 0 || angular_velocity.timestamp == 0
	    || hrt_elapsed_time(&vehicle_torque_setpoint.timestamp) > 1_s
	    || hrt_elapsed_time(&angular_velocity.timestamp) > 1_s) {
		return;
	}

	_last_control_input = math::min(vehicle_torque_setpoint.timestamp, angular_velocity.timestamp);
	perf_begin(_cycle_perf);

	const hrt_abstime timestamp_sample = vehicle_torque_setpoint.timestamp;

	// collect sample interval average for filters
	if (_last_run > 0) {
		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((timestamp_sample - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_interval_sum += dt;
		_interval_count++;

	} else {
		_interval_sum = 0.f;
		_interval_count = 0.f;
	}

	_last_run = timestamp_sample;

	checkFilters();

	// Send data to the filters at maximum frequency
	if (_state == state::roll) {
		_sys_id.updateFilters(_input_scale * vehicle_torque_setpoint.xyz[0],
				      angular_velocity.xyz[0]);

	} else if (_state == state::pitch) {
		_sys_id.updateFilters(_input_scale * vehicle_torque_setpoint.xyz[1],
				      angular_velocity.xyz[1]);

	} else if (_state == state::yaw) {
		_sys_id.updateFilters(_input_scale * vehicle_torque_setpoint.xyz[2],
				      angular_velocity.xyz[2]);
	}

	// Update the model at a lower frequency
	_model_update_counter++;

	if (_model_update_counter >= _model_update_scaler) {
		if ((_state == state::roll) || (_state == state::pitch) || (_state == state::yaw)) {
			_sys_id.update();
			_last_model_update = hrt_absolute_time();
		}

		_model_update_counter = 0;
	}

	if (hrt_elapsed_time(&_last_publish) > _publishing_dt_hrt || _last_publish == 0) {
		updateStateMachine(now);

		Vector<float, 5> coeff = _sys_id.getCoefficients();
		coeff(2) *= _input_scale;
		coeff(3) *= _input_scale;
		coeff(4) *= _input_scale;

		const Vector3f num(coeff(2), coeff(3), coeff(4));
		const Vector3f den(1.f, coeff(0), coeff(1));

		const float model_dt = static_cast<float>(_model_update_scaler) * _filter_dt;

		const float desired_rise_time = ((_state == state::yaw)
						 || (_state == state::yaw_pause)) ? 0.2f : _param_mc_at_rise_time.get();
		_kid = pid_design::computePidGmvc(num, den, model_dt, desired_rise_time, 0.f, 0.7f);

		// Prevent the D term from going just negative if it is not needed
		if ((_kid(2) < 0.f) && (_kid(2) > -0.001f)) {
			_kid(2) = 0.f;
		}

		// To compute the attitude gain, use the following empirical rule:
		// "An error of 60 degrees should produce the maximum control output"
		// or K_att * K_rate * rad(60) = 1
		_attitude_p = math::constrain(1.f / (math::radians(60.f) * _kid(0)), 2.f, 6.5f);

		const Vector<float, 5> &coeff_var = _sys_id.getVariances();

		const Vector3f rate_sp = _sys_id.areFiltersInitialized()
					 ? getIdentificationSignal()
					 : Vector3f();

		autotune_attitude_control_status_s status{};
		status.timestamp = now;
		coeff.copyTo(status.coeff);
		coeff_var.copyTo(status.coeff_var);
		status.fitness = _sys_id.getFitness();
		status.dt_model = model_dt;
		status.innov = _sys_id.getInnovation();
		status.u_filt = _sys_id.getFilteredInputData();
		status.y_filt = _sys_id.getFilteredOutputData();
		status.kc = _kid(0);
		status.ki = _kid(1);
		status.kd = _kid(2);
		status.att_p = _attitude_p;
		rate_sp.copyTo(status.rate_sp);
		status.state = static_cast<int>(_state);
		_session.publish(status);

		_last_publish = now;
	}

	perf_end(_cycle_perf);
}

void McAutotuneAttitudeControl::checkFilters()
{
	if (_interval_count > 1000) {
		// calculate sensor update rate
		_sample_interval_avg = _interval_sum / _interval_count;

		// check if sample rate error is greater than 1%
		bool reset_filters = false;

		if ((fabsf(_filter_dt - _sample_interval_avg) / _filter_dt) > 0.01f) {
			reset_filters = true;
		}

		if (reset_filters || !_are_filters_initialized) {
			_filter_dt = _sample_interval_avg;

			const float filter_rate_hz = 1.f / _filter_dt;

			_sys_id.setLpfCutoffFrequency(filter_rate_hz, _param_imu_gyro_cutoff.get());
			_sys_id.setHpfCutoffFrequency(filter_rate_hz, .5f);

			// Set the model sampling time depending on the gyro cutoff frequency
			// as this is a good indicator of the maximum control loop bandwidth
			float model_dt = math::constrain(math::max(1.f / (2.f * _param_imu_gyro_cutoff.get()), _filter_dt), _model_dt_min,
							 _model_dt_max);

			_model_update_scaler = math::max(int(model_dt / _filter_dt), 1);
			model_dt = _model_update_scaler * _filter_dt;

			_sys_id.setForgettingFactor(60.f, model_dt);
			_sys_id.setFitnessLpfTimeConstant(1.f, model_dt);

			_are_filters_initialized = true;
		}

		// reset sample interval accumulator
		_last_run = 0;
	}
}


void McAutotuneAttitudeControl::updateVehicleStatus()
{
	// copy() also detects an unavailable topic; do not retain an eligible cached status.
	if (!_vehicle_status_sub.copy(&_vehicle_status)) {
		_vehicle_status = {};
	}

	_armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	_nav_state = _vehicle_status.nav_state;
}

void McAutotuneAttitudeControl::publishState(hrt_abstime now)
{
	autotune_attitude_control_status_s status{};
	status.timestamp = now;
	status.state = static_cast<uint8_t>(_state);
	_session.publish(status);
}

void McAutotuneAttitudeControl::abortAutotune(hrt_abstime now, const char *reason)
{
	const int axis = _state == state::roll ? 0 : _state == state::pitch ? 1 : _state == state::yaw ? 2 : -1;

	if (axis >= 0) {
		_session.axis(axis, reason, true);
	}

	_session.message(reason, true);
	revertParamGains();
	_state = state::fail;
	_state_start_time = now;
	_vehicle_cmd_start_autotune = false;
	publishState(now); // zero excitation before relinquishing ownership
}

bool McAutotuneAttitudeControl::checkAbort(hrt_abstime now)
{
	const bool eligible = autotune::eligible(_vehicle_status, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, now);
	const bool active = _state != state::idle && _state != state::complete && _state != state::fail;

	if (!eligible) {
		_vehicle_cmd_start_autotune = false;

		if (active) {
			abortAutotune(now, "FAIL: flight regime changed or unavailable");
		}

		// A terminating tuner must never overwrite a new owner's status.
		_session.release();
		return active;
	}

	if (active && _state != state::wait_for_disarm) {
		manual_control_setpoint_s manual_control_setpoint{};
		_manual_control_setpoint_sub.copy(&manual_control_setpoint);
		const bool timeout = (now - _state_start_time) > 20_s;
		const bool input_lost = now - _last_control_input > 1_s;
		const bool mode_changed = (_start_flight_mode != _nav_state);
		const bool pilot_intervention = fabsf(manual_control_setpoint.roll) > 0.05f
						|| fabsf(manual_control_setpoint.pitch) > 0.05f;

		if (timeout || input_lost || mode_changed || pilot_intervention) {
			abortAutotune(now, timeout ? "FAIL: timeout" : input_lost ? "FAIL: controller input lost" :
				      mode_changed ? "FAIL: flight mode changed" : "FAIL: pilot intervention");
			return true;
		}
	}

	return false;
}

void McAutotuneAttitudeControl::updateStateMachine(hrt_abstime now)
{
	updateVehicleStatus();
	now = math::max(now, hrt_absolute_time());

	if (checkAbort(now)) {
		return;
	}

	// when identifying an axis, check if the estimate has converged
	const float converged_thr = 50.f;

	switch (_state) {
	case state::idle:
		if (_vehicle_cmd_start_autotune) {
			// Consume requests once. A busy channel or rejected regime is not a deferred start.
			_vehicle_cmd_start_autotune = false;

			if (autotune::eligible(_vehicle_status, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, now)
			    && _session.acquire(now, _start_request_timestamp)) {
				_gains_backup_available = false;
				_last_control_input = now;
				_last_publish = 0;
				_state = registerActuatorControlsCallback() ? state::init : state::fail;
				_session.message(_state == state::init ? "started; initializing" : "FAIL: callback registration", _state == state::fail);
				_state_start_time = now;
				_start_flight_mode = _nav_state;
			}
		}

		break;

	case state::init:
		if (_are_filters_initialized) {
			_state = state::roll;
			_session.axis(0, "tuning");
			_state_start_time = now;
			_sys_id.reset();
			// first step needs to be shorter to keep the drone centered
			_steps_counter = 5;
			_max_steps = 10;
			_signal_sign = 1;
			_input_scale = 1.f / (_param_mc_rollrate_p.get() * _param_mc_rollrate_k.get());
			_signal_filter.reset(0.f);
			_gains_backup_available = false;
		}

		break;

	case state::roll:
		if (areAllSmallerThan(_sys_id.getVariances(), converged_thr)
		    && ((now - _state_start_time) > 5_s)) {
			copyGains(0);
			_session.axis(0, "PASS: identification");

			// wait for the drone to stabilize
			_state = state::roll_pause;
			_state_start_time = now;
		}

		break;

	case state::roll_pause:
		if ((now - _state_start_time) > 2_s) {
			_state = state::pitch;
			_session.axis(1, "tuning");
			_state_start_time = now;
			_sys_id.reset();
			_input_scale = 1.f / (_param_mc_pitchrate_p.get() * _param_mc_pitchrate_k.get());
			_signal_filter.reset(0.f);
			_signal_sign = 1;
			// first step needs to be shorter to keep the drone centered
			_steps_counter = 5;
			_max_steps = 10;
		}

		break;

	case state::pitch:
		if (areAllSmallerThan(_sys_id.getVariances(), converged_thr)
		    && ((now - _state_start_time) > 5_s)) {
			copyGains(1);
			_session.axis(1, "PASS: identification");
			_state = state::pitch_pause;
			_state_start_time = now;
		}

		break;

	case state::pitch_pause:
		if ((now - _state_start_time) > 2_s) {
			_state = state::yaw;
			_session.axis(2, "tuning");
			_state_start_time = now;
			_sys_id.reset();
			_input_scale = 1.f / (_param_mc_yawrate_p.get() * _param_mc_yawrate_k.get());
			_signal_filter.reset(0.f);
			_signal_sign = 1;
			// first step needs to be shorter to keep the drone centered
			_steps_counter = 5;
			_max_steps = 10;
		}

		break;

	case state::yaw:
		if (areAllSmallerThan(_sys_id.getVariances(), converged_thr)
		    && ((now - _state_start_time) > 5_s)) {
			copyGains(2);
			_session.axis(2, "PASS: identification");
			_state = state::yaw_pause;
			_state_start_time = now;
		}

		break;

	case state::yaw_pause:
		if ((now - _state_start_time) > 2_s) {
			_state = state::verification;
			_state_start_time = now;
			_sys_id.reset();
			_signal_filter.reset(0.f);
			_signal_sign = 1;
			_steps_counter = 5;
			_max_steps = 10;
		}

		break;

	case state::verification:
		_state = areGainsGood()
			 ? state::apply
			 : state::fail;

		_session.message(_state == state::apply ? "gains verified" : "FAIL: gain validation", _state == state::fail);
		_state_start_time = now;
		break;

	case state::apply:
		if ((_param_mc_at_apply.get() == 1)) {
			_state = state::wait_for_disarm;
			_session.message("waiting for disarm to apply gains");

		} else if (_param_mc_at_apply.get() == 2) {
			_session.message("applying trial gains");
			backupAndSaveGainsToParams();
			_state = state::test;
			_session.message("testing trial gains");

		} else {
			_session.message("complete; gains not applied");
			_state = state::complete;
		}

		_state_start_time = now;

		break;

	case state::wait_for_disarm:
		if (!_armed) {
			_session.message("applying gains after disarm");
			saveGainsToParams();
			_session.message("complete; gain writes finished");
			_state = state::complete;
			_state_start_time = now;
		}

		break;

	case state::test:
		if ((now - _state_start_time) > 4_s) {
			_session.message("test PASS; tune complete");
			_state = state::complete;
			_gains_backup_available = false; // the trial gains have passed validation
			_state_start_time = now;

		} else if ((now - _state_start_time) < 4_s
			   && (now - _state_start_time) > 1_s
			   && _control_power.longerThan(0.1f)) {
			_session.message("test FAIL: control power; reverting", true);
			_state = state::fail;
			revertParamGains();
			_state_start_time = now;
		}

		break;

	case state::complete:

	// fallthrough
	case state::fail:

		// Wait a bit in that state to make sure
		// the other components are aware of the final result
		if ((now - _state_start_time) > 2_s) {
			_state = state::idle;
			stopAutotune();
		}

		break;
	}

}

void McAutotuneAttitudeControl::backupAndSaveGainsToParams()
{
	float backup_gains[15] = {};
	backup_gains[0] = _param_mc_rollrate_k.get();
	backup_gains[1] = _param_mc_rollrate_p.get();
	backup_gains[2] = _param_mc_rollrate_i.get();
	backup_gains[3] = _param_mc_rollrate_d.get();
	backup_gains[4] = _param_mc_roll_p.get();
	backup_gains[5] = _param_mc_pitchrate_k.get();
	backup_gains[6] = _param_mc_pitchrate_p.get();
	backup_gains[7] = _param_mc_pitchrate_i.get();
	backup_gains[8] = _param_mc_pitchrate_d.get();
	backup_gains[9] = _param_mc_pitch_p.get();
	backup_gains[10] = _param_mc_yawrate_k.get();
	backup_gains[11] = _param_mc_yawrate_p.get();
	backup_gains[12] = _param_mc_yawrate_i.get();
	backup_gains[13] = _param_mc_yawrate_d.get();
	backup_gains[14] = _param_mc_yaw_p.get();

	saveGainsToParams();

	_rate_k(0) = backup_gains[0] * backup_gains[1]; // convert and save as standard form
	_rate_i(0) = backup_gains[2] / backup_gains[1];
	_rate_d(0) = backup_gains[3] / backup_gains[1];
	_att_p(0) = backup_gains[4];
	_rate_k(1) = backup_gains[5] * backup_gains[6];
	_rate_i(1) = backup_gains[7] / backup_gains[6];
	_rate_d(1) = backup_gains[8] / backup_gains[6];
	_att_p(1) = backup_gains[9];
	_rate_k(2) = backup_gains[10] * backup_gains[11];
	_rate_i(2) = backup_gains[12] / backup_gains[11];
	_rate_d(2) = backup_gains[13] / backup_gains[11];
	_att_p(2) = backup_gains[14];

	_gains_backup_available = true;
}

void McAutotuneAttitudeControl::revertParamGains()
{
	if (_gains_backup_available) {
		_session.message("restoring previous gains", true);
		saveGainsToParams(true);
		_gains_backup_available = false;
	}
}

bool McAutotuneAttitudeControl::registerActuatorControlsCallback()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

bool McAutotuneAttitudeControl::areAllSmallerThan(const Vector<float, 5> &vect, float threshold) const
{
	return (vect(0) < threshold)
	       && (vect(1) < threshold)
	       && (vect(2) < threshold)
	       && (vect(3) < threshold)
	       && (vect(4) < threshold);
}

void McAutotuneAttitudeControl::copyGains(int index)
{
	if (index <= 2) {
		_rate_k(index) = _kid(0);
		_rate_i(index) = _kid(1);
		_rate_d(index) = _kid(2);
		_att_p(index) = _attitude_p;
	}
}

bool McAutotuneAttitudeControl::areGainsGood() const
{
	const bool are_positive = _rate_k.min() > 0.f
				  && _rate_i.min() > 0.f
				  && _rate_d.min() >= 0.f
				  && _att_p.min() > 0.f;

	const bool are_small_enough = _rate_k.max() < 0.5f
				      && _rate_i.max() < 10.f
				      && _rate_d.max() < 0.1f
				      && _att_p.max() < 12.f;

	return are_positive && are_small_enough;
}

void McAutotuneAttitudeControl::saveGainsToParams(bool restoring)
{
	// save as parallel form
	_param_mc_rollrate_p.set(_rate_k(0));
	_param_mc_rollrate_k.set(1.f);
	_param_mc_rollrate_i.set(_rate_k(0) * _rate_i(0));
	_param_mc_rollrate_d.set(_rate_k(0) * _rate_d(0));
	_param_mc_roll_p.set(_att_p(0));
	_session.writeParameter(_param_mc_rollrate_p.handle(), _param_mc_rollrate_p.get(), false, restoring);
	_session.writeParameter(_param_mc_rollrate_k.handle(), _param_mc_rollrate_k.get(), false, restoring);
	_session.writeParameter(_param_mc_rollrate_i.handle(), _param_mc_rollrate_i.get(), false, restoring);
	_session.writeParameter(_param_mc_rollrate_d.handle(), _param_mc_rollrate_d.get(), false, restoring);
	_session.writeParameter(_param_mc_roll_p.handle(), _param_mc_roll_p.get(), false, restoring);

	_param_mc_pitchrate_p.set(_rate_k(1));
	_param_mc_pitchrate_k.set(1.f);
	_param_mc_pitchrate_i.set(_rate_k(1) * _rate_i(1));
	_param_mc_pitchrate_d.set(_rate_k(1) * _rate_d(1));
	_param_mc_pitch_p.set(_att_p(1));
	_session.writeParameter(_param_mc_pitchrate_p.handle(), _param_mc_pitchrate_p.get(), false, restoring);
	_session.writeParameter(_param_mc_pitchrate_k.handle(), _param_mc_pitchrate_k.get(), false, restoring);
	_session.writeParameter(_param_mc_pitchrate_i.handle(), _param_mc_pitchrate_i.get(), false, restoring);
	_session.writeParameter(_param_mc_pitchrate_d.handle(), _param_mc_pitchrate_d.get(), false, restoring);
	_session.writeParameter(_param_mc_pitch_p.handle(), _param_mc_pitch_p.get(), false, restoring);

	_param_mc_yawrate_p.set(_rate_k(2));
	_param_mc_yawrate_k.set(1.f);
	_param_mc_yawrate_i.set(_rate_k(2) * _rate_i(2));
	_param_mc_yawrate_d.set(_rate_k(2) * _rate_d(2));
	_param_mc_yaw_p.set(_att_p(2));
	_session.writeParameter(_param_mc_yawrate_p.handle(), _param_mc_yawrate_p.get(), false, restoring);
	_session.writeParameter(_param_mc_yawrate_k.handle(), _param_mc_yawrate_k.get(), false, restoring);
	_session.writeParameter(_param_mc_yawrate_i.handle(), _param_mc_yawrate_i.get(), false, restoring);
	_session.writeParameter(_param_mc_yawrate_d.handle(), _param_mc_yawrate_d.get(), false, restoring);
	_session.writeParameter(_param_mc_yaw_p.handle(), _param_mc_yaw_p.get(), true, restoring);
}

void McAutotuneAttitudeControl::stopAutotune()
{
	_vehicle_cmd_start_autotune = false;
}

const Vector3f McAutotuneAttitudeControl::getIdentificationSignal()
{
	if (_steps_counter > _max_steps) {
		_signal_sign = (_signal_sign == 1) ? 0 : 1;
		_steps_counter = 0;

		if (_max_steps > 1) {
			_max_steps--;

		} else {
			_max_steps = 5;
		}
	}

	_steps_counter++;

	const float step = float(_signal_sign) * _param_mc_at_sysid_amp.get();

	Vector3f rate_sp{};

	const float signal = step - _signal_filter.getState();

	if (_state == state::roll) {
		rate_sp(0) = signal;

	} else if (_state ==  state::pitch) {
		rate_sp(1) = signal;

	} else if (_state ==  state::yaw) {
		rate_sp(2) = signal;

	} else if (_state == state::test) {
		rate_sp(0) = signal;
		rate_sp(1) = signal;
	}

	_signal_filter.update(step);

	return rate_sp;
}

int McAutotuneAttitudeControl::task_spawn(int argc, char *argv[])
{
	McAutotuneAttitudeControl *instance = new McAutotuneAttitudeControl();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int McAutotuneAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int McAutotuneAttitudeControl::print_status()
{
	perf_print_counter(_cycle_perf);

	return 0;
}

int McAutotuneAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_autotune_attitude_control", "autotune");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_autotune_attitude_control_main(int argc, char *argv[])
{
	return ModuleBase::main(McAutotuneAttitudeControl::desc, argc, argv);
}
