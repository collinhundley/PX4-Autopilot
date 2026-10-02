/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
* @file tailsitter.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#include "tailsitter.h"
#include <lib/rate_control/tailsitter_handoff.hpp>
#include <uORB/topics/tailsitter_handoff_status.h>
#include "vtol_att_control_main.h"

using namespace matrix;

Tailsitter::Tailsitter(VtolAttitudeControl *attc) :
	VtolType(attc)
{
	_handoff_pub.advertise();
}

void
Tailsitter::parameters_update()
{
	VtolType::updateParams();

}

Eulerf Tailsitter::getFixedWingAttitudeEuler() const
{
	// Tailsitter attitude is estimated in MC frame; rotate it to FW frame before checking FW limits.
	return Eulerf(Quatf(_v_att->q) * _q_fw_to_mc);
}

bool Tailsitter::isPitchExceeded()
{
	if (_common_vtol_mode != mode::FIXED_WING) {
		return false;
	}

	// fixed-wing maximum pitch angle
	if (_param_vt_fw_qc_p.get() > 0) {
		const Eulerf euler = getFixedWingAttitudeEuler();

		if (fabsf(euler.theta()) > fabsf(math::radians(static_cast<float>(_param_vt_fw_qc_p.get())))) {
			return true;
		}
	}

	return false;
}

bool Tailsitter::isRollExceeded()
{
	if (_common_vtol_mode != mode::FIXED_WING) {
		return false;
	}

	// fixed-wing maximum roll angle
	if (_param_vt_fw_qc_r.get() > 0) {
		const Eulerf euler = getFixedWingAttitudeEuler();

		if (fabsf(euler.phi()) > fabsf(math::radians(static_cast<float>(_param_vt_fw_qc_r.get())))) {
			return true;
		}
	}

	return false;
}

void Tailsitter::startHandoff(bool to_mc)
{
	_handoff = {};
	_handoff_active_timestamp = 0;
	_handoff_motor_residual.zero();

	if (to_mc) { _handoff_fw_sub.copy(&_handoff); }

	else { _handoff_mc_sub.copy(&_handoff); }

	// Snapshot the command actually routed to the motors, not an unused/new MC sample.
	Vector3f(_torque_setpoint_0->xyz).copyTo(_handoff.torque);
	_handoff.thrust = -_thrust_setpoint_0->xyz[2];
	_handoff.timestamp = hrt_absolute_time();
	_handoff.handoff_id = to_mc ? _mc_takeover_timestamp : _trans_finished_ts;
	_handoff.to_mc = to_mc;
	_handoff.active = false;
	_handoff.differential_thrust_scale[0] = (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::YAW_BIT)) ?
						_param_vt_fw_difthr_s_y.get() : 0.f;
	_handoff.differential_thrust_scale[1] = (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::PITCH_BIT)) ?
						_param_vt_fw_difthr_s_p.get() : 0.f;
	_handoff.differential_thrust_scale[2] = (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::ROLL_BIT)) ?
						_param_vt_fw_difthr_s_r.get() : 0.f;

	if (to_mc) {
		matrix::constrain(Vector3f(_handoff.torque_bias).emult(Vector3f(_handoff.differential_thrust_scale)), -1.f, 1.f)
		.copyTo(_handoff.torque_bias);
	}

	_handoff_pub.publish(_handoff);
}

void Tailsitter::cancelHandoff()
{
	const bool to_mc = _handoff.to_mc;
	_handoff = {};
	_handoff.to_mc = to_mc;
	_handoff.timestamp = hrt_absolute_time();
	_handoff_pub.publish(_handoff);
}

bool Tailsitter::backHandoffSafe()
{
	// A pilot's ordinary mode switch can also be an upset bailout. Do not delay recovery.
	vehicle_angular_velocity_s angular{};
	_angular_velocity_sub.copy(&angular);
	const Quatf attitude(_v_att->q);
	const Quatf target = (_vtol_mode == vtol_mode::FW_MODE
			      || (_vtol_mode == vtol_mode::TRANSITION_BACK && !_flag_was_in_trans_mode))
			     ? Quatf(_fw_virtual_att_sp->q_d) * _q_fw_to_mc.inversed() : Quatf(_v_att_sp->q_d);
	Vector3f rate_error(angular.xyz);

	if (_vtol_mode == vtol_mode::MC_MODE) {
		// Ordinary MC braking can deliberately command high pitch rates. After the
		// transition, judge loss of tracking rather than that commanded manoeuvre.
		tailsitter_handoff_s mc_state{};
		_handoff_mc_sub.copy(&mc_state);
		rate_error -= Vector3f(mc_state.rates);
	}

	const float attitude_error = 2.f * acosf(math::constrain(fabsf((attitude.inversed() * target)(0)), 0.f, 1.f));
	return _v_control_mode->flag_armed && _v_control_mode->flag_control_rates_enabled
	       && _v_control_mode->flag_control_attitude_enabled && !_land_detected->landed
	       && !_vtol_vehicle_status->fixed_wing_system_failure && !_attc->get_immediate_transition()
	       && angular.timestamp != 0 && hrt_elapsed_time(&angular.timestamp) < tailsitter_handoff::kOutputMaxAge
	       && rate_error.isAllFinite() && rate_error.norm() < math::radians(90.f)
	       && attitude.isAllFinite() && target.isAllFinite()
	       && fabsf(attitude.norm() - 1.f) < .01f && fabsf(target.norm() - 1.f) < .01f
	       && fabsf(attitude_error) < math::radians(45.f);
}

void Tailsitter::startBackHandoff()
{
	tailsitter_handoff_s source{};
	_handoff_fw_sub.copy(&source);
	control_allocator_status_s allocation{};
	_allocator_sub.copy(&allocation);
	const bool saturated = hrt_elapsed_time(&allocation.timestamp) < tailsitter_handoff::kOutputMaxAge
			       && !allocation.torque_setpoint_achieved;
	_mc_takeover_timestamp = hrt_absolute_time();

	if (!backHandoffSafe() || saturated || source.timestamp_sample == 0
	    || hrt_elapsed_time(&source.timestamp_sample) >= tailsitter_handoff::kOutputMaxAge
	    || source.timestamp_setpoint == 0 || hrt_elapsed_time(&source.timestamp_setpoint) >= tailsitter_handoff::kOutputMaxAge
	    || !Vector3f(source.torque_bias).isAllFinite() || !Vector3f(source.rates).isAllFinite()
	    || !Vector3f(_torque_setpoint_0->xyz).isAllFinite() || !PX4_ISFINITE(_thrust_setpoint_0->xyz[2])) {
		cancelHandoff();
		return;
	}

	// Reuse the motor-space snapshot and differential-thrust scales of the forward handoff.
	_back_transition_end_timestamp = 0;
	startHandoff(true);
}

void Tailsitter::update_vtol_state()
{
	const vtol_mode previous_mode = _vtol_mode;
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/


	if (_vtol_vehicle_status->fixed_wing_system_failure) {
		// Failsafe event, switch to MC mode immediately
		if (_vtol_mode != vtol_mode::MC_MODE) {
			_transition_start_timestamp = hrt_absolute_time();
			_mc_takeover_timestamp = _transition_start_timestamp;
		}

		_vtol_mode = vtol_mode::MC_MODE;

	} else if (!_attc->is_fixed_wing_requested()) {

		switch (_vtol_mode) { // user switchig to MC mode
		case vtol_mode::MC_MODE:
			break;

		case vtol_mode::FW_MODE:
			startBackHandoff();
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_BACK;
			break;

		case vtol_mode::TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_BACK:
			const float pitch = Eulerf(Quatf(_v_att->q)).theta();

			// check if we have reached pitch angle to switch to MC mode
			if (pitch >= PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC || _time_since_trans_start > _param_vt_b_trans_dur.get()) {
				_vtol_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {  // user switchig to FW mode

		switch (_vtol_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			_vtol_mode = vtol_mode::TRANSITION_FRONT_P1;
			resetTransitionStates();
			break;

		case vtol_mode::FW_MODE:
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {

				if (isFrontTransitionCompleted()) {
					_vtol_mode = vtol_mode::FW_MODE;
					_trans_finished_ts = hrt_absolute_time();
				}

				break;
			}

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_mode = vtol_mode::FW_MODE;
			_trans_finished_ts = hrt_absolute_time();
			break;
		}
	}

	if (_vtol_mode == vtol_mode::FW_MODE && previous_mode != vtol_mode::FW_MODE) {
		startHandoff();

	} else if (_handoff.handoff_id != 0
		   && ((_handoff.to_mc && (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1
					   || (previous_mode != vtol_mode::FW_MODE && !backHandoffSafe())))
		       || (!_handoff.to_mc && _vtol_mode != vtol_mode::FW_MODE))) {
		cancelHandoff();
	}

	if (previous_mode == vtol_mode::TRANSITION_BACK && _vtol_mode == vtol_mode::MC_MODE) {
		_back_transition_end_timestamp = hrt_absolute_time();
		_back_attitude_offset.zero();
		const Quatf mc_target(_mc_virtual_att_sp->q_d);

		if (_handoff.to_mc && _handoff.active && mc_target.isAllFinite() && fabsf(mc_target.norm() - 1.f) < .01f) {
			Quatf offset = _q_trans_sp * mc_target.inversed();
			offset.canonicalize();
			_back_attitude_offset = AxisAnglef(offset);
		}
	}

	if (_handoff.to_mc && _handoff.active && _vtol_mode == vtol_mode::MC_MODE
	    && hrt_elapsed_time(&_back_transition_end_timestamp) > tailsitter_handoff::kTorqueBlendTime * 1e6f) {
		tailsitter_handoff_status_s status{};
		_handoff_status_sub.copy(&status);

		if (status.handoff_id == _handoff.handoff_id && status.timestamp > _back_transition_end_timestamp
		    && !status.in_transition && !status.throttle_slew_active) {
			_handoff.to_mc = false; // Completed normally: retain the adapted MC integral.
			cancelHandoff();
		}
	}

	// map tailsitter specific control phases to simple control modes
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
		_common_vtol_mode = mode::ROTARY_WING;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::FW_MODE:
		_common_vtol_mode = mode::FIXED_WING;
		_flag_was_in_trans_mode = false;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
		_common_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_BACK:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Tailsitter::update_transition_state()
{
	VtolType::update_transition_state();

	const hrt_abstime now = hrt_absolute_time();

	// Back-transition must immediately publish an MC-frame attitude target, even while
	// the MC position controller is waking up. Retain FW collective until its demand arrives.
	const bool mc_setpoint_recent = _mc_virtual_att_sp->timestamp >= (now - 1_s);

	if (!mc_setpoint_recent && _vtol_mode != vtol_mode::TRANSITION_BACK) {
		return;
	}

	if (!_flag_was_in_trans_mode) {
		_flag_was_in_trans_mode = true;

		if (_vtol_mode == vtol_mode::TRANSITION_BACK) {
			// calculate rotation axis for transition.
			_q_trans_start = Quatf(_v_att->q);
			Vector3f z = -_q_trans_start.dcm_z();
			_trans_rot_axis = z.cross(Vector3f(0.f, 0.f, -1.f));

			// as heading setpoint we choose the heading given by the direction the vehicle points
			const float yaw_sp = atan2f(z(1), z(0));

			// the intial attitude setpoint for a backtransition is a combination of the current fw pitch setpoint,
			// the yaw setpoint and zero roll since we want wings level transition.
			// If for some reason the fw attitude setpoint is not recent then don't use it and assume 0 pitch
			if (_fw_virtual_att_sp->timestamp > (now - 1_s)) {
				const float pitch_body = Eulerf(Quatf(_fw_virtual_att_sp->q_d)).theta();
				_q_trans_start = Eulerf(0.f, pitch_body, yaw_sp);

			} else {
				_q_trans_start = Eulerf(0.f, 0.f, yaw_sp);
			}

			// attitude during transitions are controlled by mc attitude control so rotate the desired attitude to the
			// multirotor frame
			_q_trans_start = _q_trans_start * Quatf(Eulerf(0, -M_PI_2_F, 0));

		} else if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {
			// initial attitude setpoint for the transition should be with wings level
			const Eulerf setpoint_euler(Quatf(_mc_virtual_att_sp->q_d));
			_q_trans_start = Eulerf(0.f, setpoint_euler.theta(), setpoint_euler.psi());
			Vector3f x = Dcmf(Quatf(_v_att->q)) * Vector3f(1.f, 0.f, 0.f);
			_trans_rot_axis = -x.cross(Vector3f(0.f, 0.f, -1.f));
		}

		_q_trans_sp = _q_trans_start;
	}

	// ensure input quaternions are exactly normalized because acosf(1.00001) == NaN
	_q_trans_sp.normalize();

	// tilt angle (zero if vehicle nose points up (hover))
	const float cos_tilt = math::constrain(_q_trans_sp(0) * _q_trans_sp(0) - _q_trans_sp(1) * _q_trans_sp(1) -
					       _q_trans_sp(2) * _q_trans_sp(2) + _q_trans_sp(3) * _q_trans_sp(3), -1.f, 1.f);
	const float tilt = acosf(cos_tilt);

	if (_vtol_mode == vtol_mode::TRANSITION_FRONT_P1) {

		// calculate pitching rate - and constrain to at least 0.1s transition time
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_f_trans_dur.get(), 0.1f);

		if (tilt < M_PI_2_F - math::radians(_param_fw_psp_off.get())) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       _time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}

	} else if (_vtol_mode == vtol_mode::TRANSITION_BACK) {

		// calculate pitching rate - and constrain to at least 0.1s transition time
		const float trans_pitch_rate = M_PI_2_F / math::max(_param_vt_b_trans_dur.get(), 0.1f);

		if (tilt > 0.01f) {
			_q_trans_sp = Quatf(AxisAnglef(_trans_rot_axis,
						       _time_since_trans_start * trans_pitch_rate)) * _q_trans_start;
		}
	}

	_v_att_sp->thrust_body[2] = mc_setpoint_recent ? _mc_virtual_att_sp->thrust_body[2] : -_last_thr_in_fw_mode;

	_v_att_sp->timestamp = hrt_absolute_time();

	const Eulerf euler_sp(_q_trans_sp);
	_q_trans_sp.copyTo(_v_att_sp->q_d);
}

void Tailsitter::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust_body[0] = -_last_thr_in_mc;
}

void Tailsitter::update_mc_state()
{
	VtolType::update_mc_state();

	if (_handoff.to_mc && _handoff.active) {
		// Remove only the initial attitude mismatch when the transition trajectory ends.
		// Live MC target changes and attitude feedback remain active throughout.
		const float remaining = tailsitter_handoff::remaining(hrt_elapsed_time(&_back_transition_end_timestamp) * 1e-6f);
		const Quatf correction(AxisAnglef(_back_attitude_offset * remaining));
		(correction * Quatf(_v_att_sp->q_d)).copyTo(_v_att_sp->q_d);
	}
}

void Tailsitter::update_fw_state()
{
	VtolType::update_fw_state();

}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	const Vector3f previous_torque(_torque_setpoint_0->xyz);
	const float previous_thrust = _thrust_setpoint_0->xyz[2];
	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	// Motors
	if (_vtol_mode == vtol_mode::FW_MODE) {

		_thrust_setpoint_0->xyz[2] = -_vehicle_thrust_setpoint_virtual_fw->xyz[0];

		/* allow differential thrust if enabled */
		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::YAW_BIT)) {
			_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0] * _param_vt_fw_difthr_s_y.get();
		}

		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::PITCH_BIT)) {
			_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1] * _param_vt_fw_difthr_s_p.get();
		}

		if (_param_vt_fw_difthr_en.get() & static_cast<int32_t>(VtFwDifthrEnBits::ROLL_BIT)) {
			_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2] * _param_vt_fw_difthr_s_r.get();
		}

		// Accept only a matched controller generation with both outputs present. A timer cannot
		// distinguish a delayed publication from a fresh zero/uninitialized FW command.
		tailsitter_handoff_s ack{};
		_handoff_ack_sub.copy(&ack);
		const bool outputs_valid = Vector3f(_vehicle_torque_setpoint_virtual_fw->xyz).isAllFinite()
					   && PX4_ISFINITE(_vehicle_thrust_setpoint_virtual_fw->xyz[0])
					   && hrt_elapsed_time(&_vehicle_torque_setpoint_virtual_fw->timestamp) < tailsitter_handoff::kOutputMaxAge
					   && hrt_elapsed_time(&_vehicle_thrust_setpoint_virtual_fw->timestamp) < tailsitter_handoff::kOutputMaxAge;
		const bool ready = outputs_valid && (tailsitter_handoff::ready(_handoff.handoff_id, ack.handoff_id,
						     ack.timestamp_sample, _vehicle_torque_setpoint_virtual_fw->timestamp_sample,
						     _vehicle_thrust_setpoint_virtual_fw->timestamp_sample)
						     || ((!_v_control_mode->flag_control_rates_enabled || !_v_control_mode->flag_armed || _land_detected->landed)
								     && _vehicle_torque_setpoint_virtual_fw->timestamp > _handoff.handoff_id
								     && _vehicle_thrust_setpoint_virtual_fw->timestamp > _handoff.handoff_id));

		if (!_handoff.active) {
			_handoff_motor_residual = Vector3f(_handoff.torque) - Vector3f(_torque_setpoint_0->xyz);
			_thrust_setpoint_0->xyz[2] = -_handoff.thrust;
			Vector3f(_handoff.torque).copyTo(_torque_setpoint_0->xyz);

			if (ready) {
				_handoff_active_timestamp = hrt_absolute_time();
				_last_valid_fw_output = _handoff_active_timestamp;
				_handoff.active = true;
				_handoff.timestamp = hrt_absolute_time();
				_handoff_pub.publish(_handoff);

			} else if (hrt_elapsed_time(&_trans_finished_ts) > tailsitter_handoff::kOutputTimeout && _v_control_mode->flag_armed) {
				_attc->quadchute(QuadchuteReason::TransitionTimeout);
			}

		} else if (!outputs_valid) {
			previous_torque.copyTo(_torque_setpoint_0->xyz);
			_thrust_setpoint_0->xyz[2] = previous_thrust;

			if (hrt_elapsed_time(&_last_valid_fw_output) > tailsitter_handoff::kOutputTimeout) {
				_attc->quadchute(QuadchuteReason::TransitionTimeout);
			}

		} else {
			_last_valid_fw_output = hrt_absolute_time();
			const float elapsed = hrt_elapsed_time(&_handoff_active_timestamp) * 1e-6f;
			matrix::constrain(Vector3f(_torque_setpoint_0->xyz)
					  + tailsitter_handoff::remaining(elapsed) * _handoff_motor_residual, -1.f, 1.f).copyTo(_torque_setpoint_0->xyz);
		}

		_last_thr_in_fw_mode = -_thrust_setpoint_0->xyz[2];

	} else {
		_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2];

		tailsitter_handoff_s mc_state{};
		_handoff_mc_sub.copy(&mc_state);
		const bool fresh_mc = mc_state.timestamp_setpoint > _mc_takeover_timestamp
				      && _vehicle_thrust_setpoint_virtual_mc->timestamp_sample >= mc_state.timestamp_sample
				      && PX4_ISFINITE(_vehicle_thrust_setpoint_virtual_mc->xyz[2]);

		// Recovery bypasses matching and slew. Retain only the pre-existing bounded protection
		// against missing MC publications, releasing it as soon as the recovery output arrives.
		if (_vtol_mode != vtol_mode::TRANSITION_FRONT_P1 && !(_handoff.to_mc && _handoff.handoff_id != 0)
		    && !fresh_mc && hrt_elapsed_time(&_transition_start_timestamp) < 50_ms) {
			_thrust_setpoint_0->xyz[2] = -_last_thr_in_fw_mode;
		}

		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0];
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1];
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2];

		if (_handoff.to_mc && _handoff.handoff_id != 0 && !_handoff.active) {
			tailsitter_handoff_s ack{};
			_handoff_ack_sub.copy(&ack);
			const bool ready = ack.to_mc && tailsitter_handoff::ready(_handoff.handoff_id, ack.handoff_id,
					   ack.timestamp_sample, _vehicle_torque_setpoint_virtual_mc->timestamp_sample,
					   _vehicle_thrust_setpoint_virtual_mc->timestamp_sample)
					   && Vector3f(_vehicle_torque_setpoint_virtual_mc->xyz).isAllFinite()
					   && PX4_ISFINITE(_vehicle_thrust_setpoint_virtual_mc->xyz[2])
					   && hrt_elapsed_time(&_vehicle_torque_setpoint_virtual_mc->timestamp) < tailsitter_handoff::kOutputMaxAge
					   && hrt_elapsed_time(&_vehicle_thrust_setpoint_virtual_mc->timestamp) < tailsitter_handoff::kOutputMaxAge;
			Vector3f(_handoff.torque).copyTo(_torque_setpoint_0->xyz);
			_thrust_setpoint_0->xyz[2] = -_handoff.thrust;

			if (ready) {
				_handoff.active = true;
				_handoff_active_timestamp = hrt_absolute_time();
				_handoff.timestamp = hrt_absolute_time();
				_handoff_pub.publish(_handoff);

			} else if (hrt_elapsed_time(&_handoff.handoff_id) > tailsitter_handoff::kOutputTimeout) {
				cancelHandoff();
				_attc->quadchute(QuadchuteReason::TransitionTimeout);
			}
		}
	}

	// Control surfaces
	if (!_param_vt_elev_mc_lock.get() || _vtol_mode != vtol_mode::MC_MODE) {
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2];
	}
}


bool Tailsitter::isFrontTransitionCompletedBase()
{
	const bool airspeed_triggers_transition = PX4_ISFINITE(_attc->get_calibrated_airspeed());

	bool transition_to_fw = false;
	const float pitch = Eulerf(Quatf(_v_att->q)).theta();

	if (pitch <= PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW) {
		if (airspeed_triggers_transition) {
			transition_to_fw = _attc->get_calibrated_airspeed() >= _param_vt_arsp_trans.get() ;

		} else {
			transition_to_fw = true;
		}
	}

	return transition_to_fw;
}

void Tailsitter::blendThrottleAfterFrontTransition(float scale)
{
	// note: MC throttle is negative (as in negative z), while FW throttle is positive (positive x)
	_v_att_sp->thrust_body[0] = scale * _v_att_sp->thrust_body[0] + (1.f - scale) * (-_last_thr_in_mc);
}
