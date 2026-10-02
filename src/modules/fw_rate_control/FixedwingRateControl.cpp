/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

#include "FixedwingRateControl.hpp"

using namespace time_literals;
using namespace matrix;

using math::interpolate;
using math::radians;

ModuleBase::Descriptor FixedwingRateControl::desc{task_spawn, custom_command, print_usage};

FixedwingRateControl::FixedwingRateControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_actuator_controls_status_pub(vtol ? ORB_ID(actuator_controls_status_1) : ORB_ID(actuator_controls_status_0)),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_fw) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_fw) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_handle_param_vt_fw_difthr_en = param_find("VT_FW_DIFTHR_EN");

	/* fetch initial parameter values */
	parameters_update();

	_rate_ctrl_status_pub.advertise();

	if (vtol) { _handoff_status_pub.advertise(); }
}

FixedwingRateControl::~FixedwingRateControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
FixedwingRateControl::parameters_update()
{
	const Vector3f rate_p = Vector3f(_param_fw_rr_p.get(), _param_fw_pr_p.get(), _param_fw_yr_p.get());
	const Vector3f rate_i = Vector3f(_param_fw_rr_i.get(), _param_fw_pr_i.get(), _param_fw_yr_i.get());
	const Vector3f rate_d = Vector3f(_param_fw_rr_d.get(), _param_fw_pr_d.get(), _param_fw_yr_d.get());

	_rate_control.setPidGains(rate_p, rate_i, rate_d);

	_rate_control.setIntegratorLimit(
		Vector3f(_param_fw_rr_imax.get(), _param_fw_pr_imax.get(), _param_fw_yr_imax.get()));

	if (_handle_param_vt_fw_difthr_en != PARAM_INVALID) {
		param_get(_handle_param_vt_fw_difthr_en, &_param_vt_fw_difthr_en);
	}


	return PX4_OK;
}

void
FixedwingRateControl::vehicle_manual_poll()
{
	if (_vcontrol_mode.flag_control_manual_enabled && _in_fw_or_transition_wo_tailsitter_transition) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (_vcontrol_mode.flag_control_rates_enabled &&
			    !_vcontrol_mode.flag_control_attitude_enabled) {

				// RATE mode we need to generate the rate setpoint from manual user inputs

				if (_vehicle_status.is_vtol_tailsitter && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
					// the rate_sp must always be published in body (hover) frame
					_rates_sp.roll = _manual_control_setpoint.yaw * radians(_param_fw_acro_z_max.get());
					_rates_sp.yaw = -_manual_control_setpoint.roll * radians(_param_fw_acro_x_max.get());

				} else {
					_rates_sp.roll = _manual_control_setpoint.roll * radians(_param_fw_acro_x_max.get());
					_rates_sp.yaw = _manual_control_setpoint.yaw * radians(_param_fw_acro_z_max.get());
				}

				_rates_sp.timestamp = hrt_absolute_time();
				_rates_sp.pitch = -_manual_control_setpoint.pitch * radians(_param_fw_acro_y_max.get());
				_rates_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f;
				_rates_sp.reset_integral = false;

				_rate_sp_pub.publish(_rates_sp);

			} else {
				// Manual/direct control, filled in FW-frame. Note that setpoints will get transformed to body frame prior publishing.
				const float airspeed_scaling_sq = _airspeed_scaling * _airspeed_scaling;

				_vehicle_torque_setpoint.xyz[0] = math::constrain(_manual_control_setpoint.roll * _param_fw_man_r_sc.get() +
								  _param_trim_roll.get() * airspeed_scaling_sq, -1.f, 1.f);
				_vehicle_torque_setpoint.xyz[1] = math::constrain(-_manual_control_setpoint.pitch * _param_fw_man_p_sc.get() +
								  _param_trim_pitch.get() * airspeed_scaling_sq, -1.f, 1.f);
				_vehicle_torque_setpoint.xyz[2] = math::constrain(_manual_control_setpoint.yaw * _param_fw_man_y_sc.get() +
								  _param_trim_yaw.get() * airspeed_scaling_sq, -1.f, 1.f);

				_vehicle_thrust_setpoint.xyz[0] = math::constrain((_manual_control_setpoint.throttle + 1.f) * .5f, 0.f, 1.f);
			}
		}
	}
}

void
FixedwingRateControl::vehicle_land_detected_poll()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

float FixedwingRateControl::get_airspeed_and_update_scaling(float dt)
{
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().calibrated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if (_param_fw_use_airspd.get() && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().calibrated_airspeed_m_s);

		if (dt > 1.f) {
			_airspeed_filter_for_torque_scaling.reset(airspeed);

		} else {
			airspeed = _airspeed_filter_for_torque_scaling.update(airspeed, dt);
		}

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the stall airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_stall.get();
		}
	}

	/*
	 * For scaling our actuators using anything less than the stall
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and it's the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */


	if (_param_fw_arsp_scale_en.get()) {
		const float min_airspeed = math::max(_param_fw_airspd_stall.get(), 0.1f);
		const float airspeed_constrained = math::max(airspeed, min_airspeed);
		_airspeed_scaling = _param_fw_airspd_trim.get() / airspeed_constrained;

	} else {
		_airspeed_scaling = 1.0f;
	}

	return airspeed;
}

void FixedwingRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if angular velocity changed
	if (_vehicle_angular_velocity_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) { //TODO rate!

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		float dt = 0.f;

		static constexpr float DT_MIN = 0.002f;
		static constexpr float DT_MAX = 0.04f;

		vehicle_angular_velocity_s vehicle_angular_velocity{};

		if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
			dt = math::constrain((vehicle_angular_velocity.timestamp_sample - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = vehicle_angular_velocity.timestamp_sample;
		}

		if (dt < DT_MIN || dt > DT_MAX) {
			const hrt_abstime time_now_us = hrt_absolute_time();
			dt = math::constrain((time_now_us - _last_run) * 1e-6f, DT_MIN, DT_MAX);
			_last_run = time_now_us;
		}

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&angular_velocity);

		_vehicle_status_sub.update(&_vehicle_status);

		Vector3f rates(angular_velocity.xyz);
		Vector3f angular_accel{angular_velocity.xyz_derivative};

		// Tailsitter: rotate setpoint from hover to fixed-wing frame (controller is in fixed-wing frame, interface in hover)
		if (_vehicle_status.is_vtol_tailsitter) {
			rates = Vector3f(-angular_velocity.xyz[2], angular_velocity.xyz[1], angular_velocity.xyz[0]);
			angular_accel = Vector3f(-angular_velocity.xyz_derivative[2], angular_velocity.xyz_derivative[1],
						 angular_velocity.xyz_derivative[0]);
		}

		const bool is_in_transition_except_tailsitter = _vehicle_status.in_transition_mode
				&& !_vehicle_status.is_vtol_tailsitter;
		const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
		_in_fw_or_transition_wo_tailsitter_transition =  is_fixed_wing || is_in_transition_except_tailsitter;

		_vehicle_control_mode_sub.update(&_vcontrol_mode);

		vehicle_land_detected_poll();
		_handoff_sub.update(&_handoff);

		if (!_vehicle_status.is_vtol_tailsitter || !_in_fw_or_transition_wo_tailsitter_transition
		    || !_vcontrol_mode.flag_armed || _landed || !_vcontrol_mode.flag_control_rates_enabled || _handoff.handoff_id == 0
		    || _handoff.to_mc) {
			_handoff_initialized = 0;
			_throttle_slew.cancel();
			_handoff_trim.zero();
			_handoff_rate_offset.zero();
			_handoff_torque_offset.zero();
			_handoff_elapsed = 0.f;
			_handoff_ack_pending = false;
			_output_saturation.zero();
		}

		vehicle_manual_poll();
		vehicle_land_detected_poll();

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}

		tailsitter_handoff_s outgoing_state{};

		if (_vcontrol_mode.flag_control_rates_enabled) {

			const float airspeed = get_airspeed_and_update_scaling(dt);

			/* reset integrals where needed */
			if (_rates_sp.reset_integral) {
				_rate_control.resetIntegral();
			}

			launch_detection_status_s launch_detection_status{};
			_launch_detection_status_sub.copy(&launch_detection_status);

			bool control_surfaces_locked = false;

			if (hrt_elapsed_time(&launch_detection_status.timestamp) < 100_ms
			    && launch_detection_status.selected_control_surface_disarmed) {
				control_surfaces_locked = true;
			}

			// Reset integrators if the aircraft is on ground or not in a state where the fw attitude controller is run
			if (_landed || !_in_fw_or_transition_wo_tailsitter_transition || control_surfaces_locked) {

				_gain_compression.reset();
				_rate_control.resetIntegral();
			}

			// Update saturation status from control allocation feedback
			// TODO: send the unallocated value directly for better anti-windup
			Vector3<bool> diffthr_enabled(
				_param_vt_fw_difthr_en & static_cast<int32_t>(VTOLFixedWingDifferentialThrustEnabledBit::ROLL_BIT),
				_param_vt_fw_difthr_en & static_cast<int32_t>(VTOLFixedWingDifferentialThrustEnabledBit::PITCH_BIT),
				_param_vt_fw_difthr_en & static_cast<int32_t>(VTOLFixedWingDifferentialThrustEnabledBit::YAW_BIT)
			);

			// saturation handling for axis controlled by differential thrust (VTOL only)
			control_allocator_status_s control_allocator_status;

			// Set saturation flags for VTOL differential thrust feature
			// If differential thrust is enabled in an axis, assume it's the only torque authority and only update saturation using matrix 0 allocating the motors.
			if (_control_allocator_status_subs[0].update(&control_allocator_status)) {
				if (_vehicle_status.is_vtol_tailsitter) {
					tailsitter_handoff::toFW(Vector3f(control_allocator_status.unallocated_torque)).copyTo(control_allocator_status.unallocated_torque);
				}

				for (size_t i = 0; i < 3; i++) {
					if (diffthr_enabled(i)) {
						_allocator_saturation(i) = control_allocator_status.unallocated_torque[i];
					}
				}
			}

			// Set saturation flags for control surface controlled axes
			if (_vehicle_status.is_vtol ? _control_allocator_status_subs[1].update(&control_allocator_status)
			    : _control_allocator_status_subs[0].update(&control_allocator_status)) {
				if (_vehicle_status.is_vtol_tailsitter) {
					tailsitter_handoff::toFW(Vector3f(control_allocator_status.unallocated_torque)).copyTo(control_allocator_status.unallocated_torque);
				}

				for (size_t i = 0; i < 3; i++) {
					if (!diffthr_enabled(i)) {
						_allocator_saturation(i) = control_allocator_status.unallocated_torque[i];
					}
				}
			}

			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			Vector3f trim(_param_trim_roll.get(), _param_trim_pitch.get(), _param_trim_yaw.get());
			trim *= _airspeed_scaling * _airspeed_scaling;

			if (airspeed < _param_fw_airspd_trim.get()) {
				trim(0) += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
						       _param_fw_dtrim_r_vmin.get(),
						       0.0f);
				trim(1) += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
						       _param_fw_dtrim_p_vmin.get(),
						       0.0f);
				trim(2) += interpolate(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
						       _param_fw_dtrim_y_vmin.get(),
						       0.0f);

			} else {
				trim(0) += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						       _param_fw_dtrim_r_vmax.get());
				trim(1) += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						       _param_fw_dtrim_p_vmax.get());
				trim(2) += interpolate(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						       _param_fw_dtrim_y_vmax.get());
			}

			if (_vcontrol_mode.flag_control_rates_enabled) {
				if (_vehicle_status.is_vtol_tailsitter && _vcontrol_mode.flag_control_attitude_enabled) {
					_virtual_rates_sp_sub.update(&_rates_sp);

				} else {
					_rates_sp_sub.update(&_rates_sp);
				}

				for (int i = 0; i < 3; ++i) {
					_rate_control.setPositiveSaturationFlag(i, _output_saturation(i) > FLT_EPSILON || _allocator_saturation(i) > FLT_EPSILON);
					_rate_control.setNegativeSaturationFlag(i, _output_saturation(i) < -FLT_EPSILON || _allocator_saturation(i) < -FLT_EPSILON);
				}

				Vector3f body_rates_setpoint = Vector3f(_rates_sp.roll, _rates_sp.pitch, _rates_sp.yaw);

				// Tailsitter: rotate setpoint from hover to fixed-wing frame (controller is in fixed-wing frame, interface in hover)
				if (_vehicle_status.is_vtol_tailsitter) {
					body_rates_setpoint = Vector3f(-_rates_sp.yaw, _rates_sp.pitch, _rates_sp.roll);
				}

				const Vector3f gain_ff(_param_fw_rr_ff.get(), _param_fw_pr_ff.get(), _param_fw_yr_ff.get());

				// Positive feedforward compensates aerodynamic damping and is scaled linearly with airspeed.
				// Negative feedforward instead weights down the setpoint acting on the P gain (turning the
				// controller into a 2-DOF controller).
				Vector3f scaled_gain_ff;

				for (int i = 0; i < 3; i++) {
					scaled_gain_ff(i) = (gain_ff(i) >= 0.f) ? gain_ff(i) / _airspeed_scaling : gain_ff(i);
				}

				_rate_control.setFeedForwardGain(scaled_gain_ff);

				const Vector3f output_gain = _gain_compression.getGains() * (_airspeed_scaling * _airspeed_scaling);
				const bool initialize_handoff = _vehicle_status.is_vtol_tailsitter
								&& _in_fw_or_transition_wo_tailsitter_transition && _vcontrol_mode.flag_armed && !_landed
								&& !_handoff.to_mc && _handoff.handoff_id != 0 && !_handoff.active && _handoff_initialized != _handoff.handoff_id
								&& _rates_sp.timestamp > _handoff.handoff_id
								&& PX4_ISFINITE(_rates_sp.thrust_body[0]) && body_rates_setpoint.isAllFinite()
								&& PX4_ISFINITE(_handoff.thrust) && Vector3f(_handoff.torque).isAllFinite()
								&& Vector3f(_handoff.torque_bias).isAllFinite() && Vector3f(_handoff.rates).isAllFinite()
								&& _handoff.timestamp_sample != 0 && _handoff.handoff_id >= _handoff.timestamp_sample
								&& _handoff.handoff_id - _handoff.timestamp_sample < tailsitter_handoff::kOutputMaxAge;
				const Vector3f motor_scale(_handoff.differential_thrust_scale);
				const Vector3f fw_scale(motor_scale(2), motor_scale(1), motor_scale(0));

				if (initialize_handoff) {
					// The bounded integral carries as much learned bias as possible. The remainder is
					// a non-integrating runtime trim; output saturation still limits the total command.
					_handoff_trim = tailsitter_handoff::initializeBias(_rate_control, Vector3f(_handoff.torque_bias),
							motor_scale, trim, output_gain, _param_fw_rll_to_yaw_ff.get());
					_handoff_rate_offset = tailsitter_handoff::toFW(Vector3f(_handoff.rates)) - body_rates_setpoint;

					_handoff_initialized = _handoff.handoff_id;
					_handoff_elapsed = 0.f;
					_handoff_ack_pending = true;
				}

				const bool matching = _handoff_initialized != 0 && _handoff_initialized == _handoff.handoff_id;
				const float remaining = matching ? tailsitter_handoff::remaining(_handoff_elapsed) : 0.f;
				body_rates_setpoint += remaining * _handoff_rate_offset;
				// Snapshot the effective steady bias before this sample integrates rate error.
				Vector3f outgoing_bias = output_gain.emult(_rate_control.getIntegral()) + _handoff_trim + trim;
				outgoing_bias(2) += _param_fw_rll_to_yaw_ff.get() * math::constrain(outgoing_bias(0), -1.f, 1.f);
				tailsitter_handoff::toMC(outgoing_bias).copyTo(outgoing_state.torque_bias);
				tailsitter_handoff::toMC(body_rates_setpoint).copyTo(outgoing_state.rates);
				const Vector3f angular_acceleration_setpoint = _rate_control.update(rates, body_rates_setpoint, angular_accel, dt,
						_landed || (matching && !_handoff.active));
				Vector3f control_u = output_gain.emult(angular_acceleration_setpoint) + _handoff_trim;

				if (!matching) { _gain_compression.update(control_u, dt); }

				// Apply the Acro direct-yaw law before matching the outgoing torque, so this
				// mode cannot bypass the transient correction. Keep the transferred bias as runtime
				// trim when this mode deliberately disables the FW yaw integrator.
				if (!_vcontrol_mode.flag_control_attitude_enabled && _vcontrol_mode.flag_control_manual_enabled
				    && !_param_fw_acro_yaw_en.get()) {
					if (initialize_handoff) {
						_handoff_trim(2) += output_gain(2) * _rate_control.getIntegral()(2);
					}

					control_u(2) = _manual_control_setpoint.yaw * _param_fw_man_y_sc.get() + _handoff_trim(2);
					_rate_control.resetIntegral(2);
				}

				if (matching && _handoff_elapsed <= FLT_EPSILON) {
					Vector3f target = tailsitter_handoff::motorToFW(Vector3f(_handoff.torque), motor_scale);
					// Roll-to-yaw feedforward uses the clipped roll output. A surface-only roll
					// axis keeps its own controller output rather than the zero motor-space target.
					const float roll_output = fw_scale(0) > FLT_EPSILON ? target(0) : control_u(0) + trim(0);
					target(2) -= _param_fw_rll_to_yaw_ff.get() * math::constrain(roll_output, -1.f, 1.f);
					_handoff_torque_offset = target - control_u - trim;

					for (int i = 0; i < 3; ++i) {
						if (fw_scale(i) <= FLT_EPSILON) { _handoff_torque_offset(i) = 0.f; }
					}
				}

				control_u += remaining * _handoff_torque_offset;
				_output_saturation = control_u + trim - matrix::constrain(control_u + trim, -1.f, 1.f);

				// Keep the matched controller state fixed until the router accepts it. Updating
				// compression while waiting would change the very first accepted FW command.
				if (matching && _handoff.active) { _gain_compression.update(control_u, dt); }

				if (control_u.isAllFinite()) {
					matrix::constrain(control_u + trim, -1.f, 1.f).copyTo(_vehicle_torque_setpoint.xyz);

				} else {
					_rate_control.resetIntegral();
					(matching ? Vector3f(NAN, NAN, NAN) : trim).copyTo(_vehicle_torque_setpoint.xyz);
				}

				float thrust_setpoint = _rates_sp.thrust_body[0];

				// Stop motor if its setpoint is below 2%. This value was determined empirically (RC stick inaccuracy).
				// Motor is stopped by setting the output to NAN (per definition).
				if (PX4_ISFINITE(thrust_setpoint) && thrust_setpoint > 0.02f) {
					/* scale effort by battery status */
					if (_param_fw_bat_scale_en.get()) {

						if (_battery_status_sub.updated()) {
							battery_status_s battery_status{};

							if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
								_battery_scale = battery_status.scale;
							}
						}

						thrust_setpoint *= _battery_scale;
					}

				} else {
					thrust_setpoint = NAN;
				}

				if (matching) {
					const float demand = PX4_ISFINITE(_rates_sp.thrust_body[0])
							     ? (PX4_ISFINITE(thrust_setpoint) ? math::constrain(thrust_setpoint, 0.f, 1.f) : 0.f) : NAN;

					if (initialize_handoff) {
						// Optional VTOL parameter; read once per handoff, never modify persistent tuning.
						float slew_rate{0.f};
						param_get(param_find("VT_TS_THR_SLEW"), &slew_rate);
						_throttle_slew.reset(_handoff.thrust, slew_rate);
					}

					_handoff_tecs_sub.update();
					const bool underspeed = _vcontrol_mode.flag_control_altitude_enabled
								&& hrt_elapsed_time(&_handoff_tecs_sub.get().timestamp) < tailsitter_handoff::kOutputMaxAge
								&& _handoff_tecs_sub.get().underspeed_ratio > FLT_EPSILON;
					const bool was_active = _throttle_slew.active();
					thrust_setpoint = _throttle_slew.update(demand,
										_handoff.active && _handoff_elapsed > FLT_EPSILON ? dt : 0.f, _rates_sp.timestamp, underspeed);

					if (was_active) {
						tailsitter_handoff_status_s feedback{};
						feedback.timestamp = hrt_absolute_time();
						feedback.handoff_id = _handoff.handoff_id;
						feedback.throttle_slew_active = _throttle_slew.active();
						feedback.thrust = thrust_setpoint;
						feedback.demand = demand;
						feedback.battery_scale = _param_fw_bat_scale_en.get() ? _battery_scale : 1.f;
						_handoff_status_pub.publish(feedback);
					}
				}

				_vehicle_thrust_setpoint.xyz[0] = thrust_setpoint;

			}

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();

			_rate_ctrl_status_pub.publish(rate_ctrl_status);

		} else {
			// full manual
			_gain_compression.reset();
			_rate_control.resetIntegral();
		}

		// Add feed-forward from roll control output to yaw control output
		// This can be used to counteract the adverse yaw effect when rolling the plane
		const float yaw_with_roll_ff = _vehicle_torque_setpoint.xyz[2] + _param_fw_rll_to_yaw_ff.get() *
					       _vehicle_torque_setpoint.xyz[0];
		_vehicle_torque_setpoint.xyz[2] = math::constrain(yaw_with_roll_ff, -1.f, 1.f);
		_output_saturation(2) += yaw_with_roll_ff - _vehicle_torque_setpoint.xyz[2];

		// Tailsitter: rotate back to body frame from airspeed frame
		if (_vehicle_status.is_vtol_tailsitter) {
			const float helper = _vehicle_torque_setpoint.xyz[0];
			_vehicle_torque_setpoint.xyz[0] = _vehicle_torque_setpoint.xyz[2];
			_vehicle_torque_setpoint.xyz[2] = -helper;
		}

		if (_handoff_initialized != 0 && _handoff_initialized == _handoff.handoff_id) {
			if (!_handoff.active && PX4_ISFINITE(_rates_sp.thrust_body[0])
			    && Vector3f(_vehicle_torque_setpoint.xyz).isAllFinite()) {
				const Vector3f scale(_handoff.differential_thrust_scale);

				for (int i = 0; i < 3; ++i) {
					if (scale(i) > FLT_EPSILON) {
						_vehicle_torque_setpoint.xyz[i] = math::constrain(_handoff.torque[i] / scale(i), -1.f, 1.f);
					}
				}

				_vehicle_thrust_setpoint.xyz[0] = _handoff.thrust;

			} else if (_handoff.active) {
				_handoff_elapsed += dt;
			}

			if (_handoff_ack_pending && !_handoff.to_mc && angular_velocity.timestamp_sample > _handoff.handoff_id
			    && Vector3f(_vehicle_torque_setpoint.xyz).isAllFinite()
			    && PX4_ISFINITE(_vehicle_thrust_setpoint.xyz[0])) {
				tailsitter_handoff_s ack{};
				ack.timestamp = hrt_absolute_time();
				ack.timestamp_sample = angular_velocity.timestamp_sample;
				ack.handoff_id = _handoff.handoff_id;
				_handoff_ack_pub.publish(ack);
				_handoff_ack_pending = false;
			}

		}

		if (_vehicle_status.is_vtol_tailsitter && is_fixed_wing && _vcontrol_mode.flag_control_attitude_enabled
		    && _vcontrol_mode.flag_control_rates_enabled) {
			outgoing_state.timestamp = hrt_absolute_time();
			outgoing_state.timestamp_sample = angular_velocity.timestamp_sample;
			outgoing_state.timestamp_setpoint = _rates_sp.timestamp;
			Vector3f(_vehicle_torque_setpoint.xyz).copyTo(outgoing_state.torque);
			outgoing_state.thrust = _vehicle_thrust_setpoint.xyz[0];
			_handoff_state_pub.publish(outgoing_state);
		}

		/* Only publish if any of the proper modes are enabled */
		if (_vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled ||
		    _vcontrol_mode.flag_control_manual_enabled) {
			{
				_vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
				_vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
				_vehicle_thrust_setpoint_pub.publish(_vehicle_thrust_setpoint);

				_vehicle_torque_setpoint.timestamp = hrt_absolute_time();
				_vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
				_vehicle_torque_setpoint_pub.publish(_vehicle_torque_setpoint);
			}
		}

		updateActuatorControlsStatus(dt);

		// Manual flaps/spoilers control, also active in VTOL Hover. Is handled and published in FW Position controller/VTOL module if Auto.
		if (_vcontrol_mode.flag_control_manual_enabled) {

			// Flaps control
			float flaps_control = 0.f; // default to no flaps

			switch (_param_fw_flaps_man.get()) { 		// do not consider negative switch settings
			case 0:
				break;

			case 1:
				flaps_control = PX4_ISFINITE(_manual_control_setpoint.aux1) ? math::max(_manual_control_setpoint.aux1, 0.f) : 0.f;
				break;

			case 2:
				flaps_control = PX4_ISFINITE(_manual_control_setpoint.aux2) ? math::max(_manual_control_setpoint.aux2, 0.f) : 0.f;
				break;

			case 3:
				flaps_control = PX4_ISFINITE(_manual_control_setpoint.aux3) ? math::max(_manual_control_setpoint.aux3, 0.f) : 0.f;
				break;

			case 4:
				flaps_control = PX4_ISFINITE(_manual_control_setpoint.aux4) ? math::max(_manual_control_setpoint.aux4, 0.f) : 0.f;
				break;

			case 5:
				flaps_control = PX4_ISFINITE(_manual_control_setpoint.aux5) ? math::max(_manual_control_setpoint.aux5, 0.f) : 0.f;
				break;

			case 6:
				flaps_control = PX4_ISFINITE(_manual_control_setpoint.flaps) ? math::max(_manual_control_setpoint.flaps, 0.f) : 0.f;
				break;


			}

			normalized_unsigned_setpoint_s flaps_setpoint;
			flaps_setpoint.timestamp = hrt_absolute_time();
			flaps_setpoint.normalized_setpoint = flaps_control;
			_flaps_setpoint_pub.publish(flaps_setpoint);

			// Spoilers control
			float spoilers_control = 0.f; // default to no spoilers

			switch (_param_fw_spoilers_man.get()) {		// do not consider negative switch settings
			case 0:
				break;

			case 1:
				spoilers_control = PX4_ISFINITE(_manual_control_setpoint.flaps) ? math::max(_manual_control_setpoint.flaps, 0.f) : 0.f;
				break;

			case 2:
				spoilers_control = PX4_ISFINITE(_manual_control_setpoint.aux1) ? math::max(_manual_control_setpoint.aux1, 0.f) : 0.f;
				break;

			case 3:
				spoilers_control = PX4_ISFINITE(_manual_control_setpoint.aux2) ? math::max(_manual_control_setpoint.aux2, 0.f) : 0.f;
				break;

			case 4:
				spoilers_control = PX4_ISFINITE(_manual_control_setpoint.aux3) ? math::max(_manual_control_setpoint.aux3, 0.f) : 0.f;
				break;

			case 5:
				spoilers_control = PX4_ISFINITE(_manual_control_setpoint.aux4) ? math::max(_manual_control_setpoint.aux4, 0.f) : 0.f;
				break;
			}

			normalized_unsigned_setpoint_s spoilers_setpoint;
			spoilers_setpoint.timestamp = hrt_absolute_time();
			spoilers_setpoint.normalized_setpoint = spoilers_control;
			_spoilers_setpoint_pub.publish(spoilers_setpoint);
		}
	}

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

void FixedwingRateControl::updateActuatorControlsStatus(float dt)
{
	for (int i = 0; i < 3; i++) {

		// We assume that the attitude is actuated by control surfaces
		// consuming power only when they move
		const float control_signal = _vehicle_torque_setpoint.xyz[i] - _control_prev[i];
		_control_prev[i] = _vehicle_torque_setpoint.xyz[i];

		_control_energy[i] += control_signal * control_signal * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = _vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int FixedwingRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingRateControl *instance = new FixedwingRateControl(vtol);

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

int FixedwingRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_rate_control is the fixed-wing rate controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_rate_control_main(int argc, char *argv[])
{
	return ModuleBase::main(FixedwingRateControl::desc, argc, argv);
}
