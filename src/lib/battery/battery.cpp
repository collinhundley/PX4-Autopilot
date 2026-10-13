/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
 * @file battery.cpp
 *
 * Library calls for battery functionality.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Timothy Scott <timothy@auterion.com>
 */

#include "battery.h"
#include <mathlib/mathlib.h>
#include <cstring>
#include <px4_platform_common/defines.h>

using namespace time_literals;
using namespace matrix;

Battery::Battery(int index, ModuleParams *parent, const int sample_interval_us, const uint8_t source) :
	ModuleParams(parent),
	_index(index < 1 || index > 9 ? 1 : index),
	_source(source)
{
	const float expected_filter_dt = static_cast<float>(sample_interval_us) / 1_s;
	_current_average_filter_a.setParameters(expected_filter_dt, 50.f);
	_ocv_filter_v.setParameters(expected_filter_dt, 1.f);
	_cell_voltage_filter_v.setParameters(expected_filter_dt, 1.f);

	if (index > 9 || index < 1) {
		PX4_ERR("Battery index must be between 1 and 9 (inclusive). Received %d. Defaulting to 1.", index);
	}

	// 16 chars for parameter name + null terminator
	char param_name[17];

	snprintf(param_name, sizeof(param_name), "BAT%d_V_EMPTY", _index);
	_param_handles.v_empty = param_find(param_name);

	if (_param_handles.v_empty == PARAM_INVALID) {
		PX4_ERR("Could not find parameter with name %s", param_name);
	}

	snprintf(param_name, sizeof(param_name), "BAT%d_V_CHARGED", _index);
	_param_handles.v_charged = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_N_CELLS", _index);
	_param_handles.n_cells = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_CAPACITY", _index);
	_param_handles.capacity = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_R_INTERNAL", _index);
	_param_handles.r_internal = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_SOURCE", _index);
	_param_handles.source = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_I_OVERWRITE", _index);
	_param_handles.i_overwrite = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_SOC_MODE", _index);
	_param_handles.soc_mode = param_find(param_name);

	for (unsigned i = 0; i < BatterySocEstimator::PROFILE_POINTS; ++i) {
		snprintf(param_name, sizeof(param_name), "BAT%d_SOC_V%u", _index, i);
		_param_handles.soc_voltage[i] = param_find(param_name);
	}

	snprintf(param_name, sizeof(param_name), "BAT%d_V_LOW", _index);
	_param_handles.voltage_low = param_find(param_name);
	snprintf(param_name, sizeof(param_name), "BAT%d_V_CRIT", _index);
	_param_handles.voltage_critical = param_find(param_name);
	snprintf(param_name, sizeof(param_name), "BAT%d_V_EMERG", _index);
	_param_handles.voltage_emergency = param_find(param_name);

	_param_handles.low_thr = param_find("BAT_LOW_THR");
	_param_handles.crit_thr = param_find("BAT_CRIT_THR");
	_param_handles.emergen_thr = param_find("BAT_EMERGEN_THR");

	_param_handles.bat_avrg_current = param_find("BAT_AVRG_CURRENT");

	updateParams();
}

void Battery::updateVoltage(const float voltage_v)
{
	_voltage_v = voltage_v;
}

void Battery::updateCurrent(const float current_a)
{
	// Overwrite the measured current if current overwrite is defined and vehicle is unarmed
	if (!_armed && _params.i_overwrite > FLT_EPSILON) {
		_current_a = _params.i_overwrite;

	} else {
		_current_a = current_a;
	}
}

void Battery::updateTemperature(const float temperature_c)
{
	_temperature_c = temperature_c;
}

void Battery::updateBatteryStatus(const hrt_abstime &timestamp)
{
	if (_last_timestamp != 0 && timestamp <= _last_timestamp) {
		return;
	}

	_sample_timestamp = timestamp;
	updateDt(timestamp);

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			_vehicle_status_is_fw = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
			_vehicle_status_timestamp = vehicle_status.timestamp;
		}
	}

	if (!_armed && _parameter_update_pending) {
		updateParams();
	}

	// Require minimum voltage otherwise override connected status.
	if (PX4_ISFINITE(_voltage_v) && _voltage_v < LITHIUM_BATTERY_RECOGNITION_VOLTAGE) {
		_connected = false;
	}

	if (_connected != _was_connected) {
		// An observed disconnect invalidates charge history, even if the same pack is reconnected.
		_discharged_mah = _discharged_mah_loop = 0.f;
		_dt_discharge = 0.f;
		_legacy_state_of_charge = -1.f;
		_soc_estimator.reset();
		_soc_active = false;
		_was_connected = _connected;
		_warning = battery_status_s::WARNING_NONE;
	}

	if (!_connected || (_last_unconnected_timestamp == 0)) {
		_last_unconnected_timestamp = timestamp;
	}

	// Wait with initializing filters to avoid relying on a voltage sample from the rising edge.
	_battery_initialized = _connected && (timestamp > _last_unconnected_timestamp + 2_s);

	if (_connected && !_battery_initialized && _internal_resistance_initialized && _params.n_cells > 0
	    && PX4_ISFINITE(_voltage_v)) {
		// Unknown current must not poison the voltage filters or introduce negative sag compensation.
		const float initialization_current = PX4_ISFINITE(_current_a) && _current_a >= 0.f ? _current_a : 0.f;
		resetInternalResistanceEstimation(_voltage_v, initialization_current);
	}

	_discharged_mah_loop = 0.f;

	if (_connected) {
		sumDischarged(_current_a);
	}

	_state_of_charge_volt_based = calculateStateOfChargeVoltageBased(_voltage_v, _current_a);

	if (!_external_state_of_charge) {
		estimateStateOfCharge();
		_state_of_charge = _legacy_state_of_charge;
	}

	if (_soc_mode != battery_soc_status_s::MODE_DISABLED && !_external_state_of_charge) {
		const bool vehicle_status_valid = _vehicle_status_timestamp != 0 && timestamp >= _vehicle_status_timestamp
						  && timestamp - _vehicle_status_timestamp < 2_s;
		// Idle current overwrite is not a measurement and cannot qualify voltage relaxation.
		const float measured_current = _params.i_overwrite > FLT_EPSILON ? NAN : _current_a;
		const float cell_voltage = _params.n_cells > 0 ? _voltage_v / _params.n_cells : NAN;
		_soc_estimator.update(timestamp, _connected, _armed, vehicle_status_valid, measured_current,
				      _battery_initialized && PX4_ISFINITE(cell_voltage) ? _cell_voltage_filter_v.getState() : NAN,
				      cell_voltage, _discharged_mah);

		if (_soc_mode == battery_soc_status_s::MODE_ACTIVE && _soc_estimator.profileValid()
		    && _soc_estimator.protectionValid() && _soc_estimator.initialized()) {
			_soc_active = true;
		}

		if (_soc_mode == battery_soc_status_s::MODE_ACTIVE) {
			// An explicitly selected active estimator must be ready before flight. Do not conceal
			// an invalid profile or missing startup voltage behind the legacy estimate.
			_state_of_charge = _soc_active && _connected && PX4_ISFINITE(_soc_estimator.soc()) ? _soc_estimator.soc() : -1.f;
		}
	}

	computeScale();

	if (_connected && _battery_initialized) {
		_warning = determineWarning(_state_of_charge);

		if (_soc_mode == battery_soc_status_s::MODE_ACTIVE && !_external_state_of_charge
		    && _soc_estimator.protectionValid()) {
			_warning = math::max(_warning, _soc_estimator.voltageWarning());
		}
	}
}

battery_status_s Battery::getBatteryStatus()
{
	battery_status_s battery_status{};
	battery_status.voltage_v = _voltage_v;
	// Publish the exact filter used by calculateStateOfChargeVoltageBased(),
	// including a configured resistance override. The diagnostic OCV filter
	// always uses estimated resistance and can differ from this value.
	const float compensated_voltage = _cell_voltage_filter_v.getState() * _params.n_cells;
	battery_status.voltage_v_compensated = _connected && _battery_initialized && _params.n_cells > 0
					       && PX4_ISFINITE(compensated_voltage) && compensated_voltage > 0.f ? compensated_voltage : 0.f;
	battery_status.current_a = _current_a;
	battery_status.current_average_a = _current_average_filter_a.getState();
	battery_status.discharged_mah = _discharged_mah;
	battery_status.remaining = _state_of_charge;
	battery_status.scale = _scale;
	battery_status.time_remaining_s = _state_of_charge >= 0.f ? computeRemainingTime(_current_a) : NAN;
	battery_status.temperature = _temperature_c;
	battery_status.cell_count = _params.n_cells;
	battery_status.connected = _connected;
	battery_status.source = _source;
	battery_status.priority = _priority;
	battery_status.capacity = static_cast<uint16_t>(_capacity_mah);
	battery_status.id = static_cast<uint8_t>(_index);
	battery_status.warning = _warning;
	battery_status.timestamp = hrt_absolute_time();
	battery_status.faults = determineFaults();
	battery_status.remaining_capacity_wh = NAN; // not measured by dumb power module; smart-battery drivers overwrite
	battery_status.nominal_voltage = NAN;        // not measured by dumb power module; smart-battery drivers overwrite
	battery_status.internal_resistance_estimate = _internal_resistance_estimate;
	battery_status.ocv_estimate = _voltage_v + _internal_resistance_estimate * _params.n_cells * _current_a;
	battery_status.ocv_estimate_filtered = _ocv_filter_v.getState();
	battery_status.volt_based_soc_estimate = _params.n_cells > 0 ? math::interpolate(_ocv_filter_v.getState() /
			_params.n_cells,
			_params.v_empty, _params.v_charged, 0.f, 1.f) : -1.f;
	battery_status.voltage_prediction = _voltage_prediction;
	battery_status.prediction_error = _prediction_error;
	battery_status.estimation_covariance_norm = _estimation_covariance_norm;
	return battery_status;
}

battery_soc_status_s Battery::getSocEstimatorStatus() const
{
	battery_soc_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.timestamp_sample = _sample_timestamp;
	status.battery_id = _index;
	status.source = _source;
	status.mode = _soc_mode;
	status.legacy_soc = _external_state_of_charge ? _state_of_charge : _legacy_state_of_charge;
	status.coulomb_soc = _soc_estimator.coulombSoc();
	status.voltage_soc = _soc_estimator.voltageSoc();
	status.soc = _soc_estimator.soc();
	status.voltage_v = _voltage_v;
	status.current_a = _current_a;
	status.discharged_mah = _discharged_mah;
	status.compensated_cell_voltage = _cell_voltage_filter_v.getState();
	status.filtered_terminal_cell_voltage = _soc_estimator.filteredTerminalVoltage();
	status.profile_valid = _soc_estimator.profileValid();
	status.protection_valid = _soc_estimator.protectionValid();
	status.initialized = _soc_estimator.initialized();
	status.charge_history_valid = _soc_estimator.chargeHistoryValid();
	status.full_charge_confirmed = _soc_estimator.fullChargeConfirmed();
	status.correction_eligible = _soc_estimator.correctionEligible();
	status.active = _soc_active && _connected && !_external_state_of_charge;
	status.external_soc = _external_state_of_charge;
	status.soc_warning = determineWarning(_state_of_charge);
	status.voltage_warning = _soc_estimator.voltageWarning();
	status.warning = _warning;
	return status;
}

void Battery::publishBatteryStatus(const battery_status_s &battery_status)
{
	if (_source == _params.source) {
		_battery_status_pub.publish(battery_status);

		// Five Hz matches the default battery logging rate; identity is carried explicitly.
		if (_soc_mode != battery_soc_status_s::MODE_DISABLED
		    && (_last_soc_publication == 0 || _sample_timestamp - _last_soc_publication >= 200_ms)) {
			_soc_status_pub.publish(getSocEstimatorStatus());
			_last_soc_publication = _sample_timestamp;
		}
	}
}

void Battery::updateAndPublishBatteryStatus(const hrt_abstime &timestamp)
{
	updateBatteryStatus(timestamp);
	publishBatteryStatus(getBatteryStatus());
}
void Battery::updateDt(const hrt_abstime &timestamp)
{
	_dt = _dt_discharge = 0.f;

	if (_last_timestamp != 0 && timestamp > _last_timestamp) {
		// _dt_discharge is the true, unclamped time delta: for the coulomb
		// count in sumDischarged() below, using the real elapsed time is
		// always more accurate than clamping it, even across an unusually
		// long gap between updates (e.g. an update source that reports
		// less often than every 2 seconds).
		_dt_discharge = (timestamp - _last_timestamp) / 1e6f;

		// _dt is clamped to guard the numerical stability of the current
		// average filter below, which assumes a roughly steady sample
		// interval and should not see a single very large dt.
		_dt = math::min(_dt_discharge, 2.f);
	}

	_last_timestamp = timestamp;
}

float Battery::sumDischarged(float current_a)
{
	_discharged_mah_loop = 0.f;

	if (_dt_discharge > FLT_EPSILON && PX4_ISFINITE(current_a) && fabsf(current_a + 1.f) > FLT_EPSILON) {
		// mAh since last loop: (current[A] * 1000 = [mA]) * (dt[s] / 3600 = [h])
		// current = -1 means invalid current measurement
		_discharged_mah_loop = (current_a * 1e3f) * (_dt_discharge / 3600.f);
		_discharged_mah += _discharged_mah_loop;
	}

	return _discharged_mah;
}

float Battery::calculateStateOfChargeVoltageBased(const float voltage_v, const float current_a)
{
	if (_params.n_cells <= 0 || !PX4_ISFINITE(voltage_v) || voltage_v <= 0.f) {
		return -1.0f;
	}

	// remaining battery capacity based on voltage
	float cell_voltage = voltage_v / _params.n_cells;

	// correct battery voltage locally for load drop according to internal resistance and current
	if (PX4_ISFINITE(current_a) && current_a > FLT_EPSILON) {
		updateInternalResistanceEstimation(voltage_v, current_a);

		if (_params.r_internal >= 0.f) { // Use user specified internal resistance value
			cell_voltage += _params.r_internal * current_a;

		} else { // Use estimated internal resistance value
			cell_voltage += _internal_resistance_estimate * current_a;
		}

	}

	if (!PX4_ISFINITE(_cell_voltage_filter_v.getState())) {
		_cell_voltage_filter_v.reset(cell_voltage);

	} else {
		_cell_voltage_filter_v.update(cell_voltage, _dt);
	}

	return math::interpolate(_cell_voltage_filter_v.getState(), _params.v_empty, _params.v_charged, 0.f, 1.f);
}

void Battery::updateInternalResistanceEstimation(const float voltage_v, const float current_a)
{
	Vector2f x{1, -current_a};
	_voltage_prediction = (x.transpose() * _RLS_est)(0, 0);
	_prediction_error = voltage_v - _voltage_prediction;
	const Vector2f gamma = _estimation_covariance * x / (LAMBDA + (x.transpose() * _estimation_covariance * x)(0, 0));
	const Vector2f RSL_est_temp = _RLS_est + gamma * _prediction_error;
	const Matrix2f estimation_covariance_temp = (_estimation_covariance
			- Matrix<float, 2, 1>(gamma) * (x.transpose() * _estimation_covariance)) / LAMBDA;
	const float estimation_covariance_temp_norm =
		sqrtf(powf(estimation_covariance_temp(0, 0), 2.f)
		      + 2.f * powf(estimation_covariance_temp(1, 0), 2.f)
		      + powf(estimation_covariance_temp(1, 1), 2.f));

	if (estimation_covariance_temp_norm < _estimation_covariance_norm) { // Only update if estimation improves
		_RLS_est = RSL_est_temp;
		_estimation_covariance = estimation_covariance_temp;
		_estimation_covariance_norm = estimation_covariance_temp_norm;
		_internal_resistance_estimate =
			math::max(_RLS_est(1) / _params.n_cells, 0.f); // Only use positive values

	} else { // Update OCV estimate with IR estimate
		_RLS_est(0) = voltage_v + _RLS_est(1) * current_a;
	}

	_ocv_filter_v.update(voltage_v + _internal_resistance_estimate * _params.n_cells * current_a);
}

void Battery::resetInternalResistanceEstimation(const float voltage_v, const float current_a)
{
	_RLS_est(0) = voltage_v;
	_RLS_est(1) = R_DEFAULT * _params.n_cells;
	_estimation_covariance.setZero();
	_estimation_covariance(0, 0) = OCV_COVARIANCE * _params.n_cells;
	_estimation_covariance(1, 1) = R_COVARIANCE * _params.n_cells;
	_estimation_covariance_norm = sqrtf(powf(_estimation_covariance(0, 0), 2.f) + 2.f * powf(_estimation_covariance(1, 0),
					    2.f) + powf(_estimation_covariance(1, 1), 2.f));
	_internal_resistance_estimate = R_DEFAULT;
	_ocv_filter_v.reset(voltage_v + _internal_resistance_estimate * _params.n_cells * current_a);

	if (_params.r_internal >= 0.f) { // Use user specified internal resistance value
		_cell_voltage_filter_v.reset(voltage_v / _params.n_cells + _params.r_internal * current_a);

	} else { // Use estimated internal resistance value
		_cell_voltage_filter_v.reset(voltage_v / _params.n_cells + _internal_resistance_estimate * current_a);
	}
}

void Battery::estimateStateOfCharge()
{
	if (!PX4_ISFINITE(_state_of_charge_volt_based) || _state_of_charge_volt_based < 0.f) {
		return;
	}

	// choose which quantity we're using for final reporting
	if ((_capacity_mah > 0.f) && _battery_initialized) {
		// if battery capacity is known, fuse voltage measurement with used capacity
		// The lower the voltage the more adjust the estimate with it to avoid deep discharge
		const float weight_v = 3e-2f * (1 - _state_of_charge_volt_based);
		_legacy_state_of_charge = (1 - weight_v) * _legacy_state_of_charge + weight_v * _state_of_charge_volt_based;
		// directly apply current capacity slope calculated using current
		_legacy_state_of_charge -= _discharged_mah_loop / _capacity_mah;
		_legacy_state_of_charge = math::max(_legacy_state_of_charge, 0.f);

		const float state_of_charge_current_based = math::max(1.f - _discharged_mah / _capacity_mah, 0.f);
		_legacy_state_of_charge = math::min(state_of_charge_current_based, _legacy_state_of_charge);

	} else {
		_legacy_state_of_charge = _state_of_charge_volt_based;
	}
}

uint8_t Battery::determineWarning(float state_of_charge) const
{
	if (!PX4_ISFINITE(state_of_charge) || state_of_charge < 0.f || state_of_charge > 1.f) {
		return battery_status_s::WARNING_NONE; // Missing SOC is an arming failure, not a low-charge measurement.
	}

	if (state_of_charge < _params.emergen_thr) {
		return battery_status_s::WARNING_EMERGENCY;

	} else if (state_of_charge < _params.crit_thr) {
		return battery_status_s::WARNING_CRITICAL;

	} else if (state_of_charge < _params.low_thr) {
		return battery_status_s::WARNING_LOW;

	} else {
		return battery_status_s::WARNING_NONE;
	}
}

uint16_t Battery::determineFaults()
{
	uint16_t faults{0};

	if ((_params.n_cells > 0)
	    && (_voltage_v > (_params.n_cells * _params.v_charged * 1.05f))) {
		// Reported as a "spike" since "over-voltage" does not exist in MAV_BATTERY_FAULT
		faults |= (1 << battery_status_s::FAULT_SPIKES);
	}

	if (PX4_ISFINITE(_temperature_c) && _temperature_c > BAT_TEMP_MAX) {
		faults |= (1 << battery_status_s::FAULT_OVER_TEMPERATURE);
	}

	return faults;
}

void Battery::computeScale()
{
	_scale = _params.v_charged / _cell_voltage_filter_v.getState();

	if (PX4_ISFINITE(_scale)) {
		_scale = math::constrain(_scale, 1.f, 1.3f); // Allow at most 30% compensation

	} else {
		_scale = 1.f;
	}
}

float Battery::computeRemainingTime(float current_a)
{
	float time_remaining_s = NAN;

	_flight_phase_estimation_sub.update();

	bool reset_filter_transition_now = !_vehicle_status_was_fw && _vehicle_status_is_fw;

	// reset filter if not feasible, negative or we did a VTOL transition to FW mode
	if (!PX4_ISFINITE(_current_average_filter_a.getState()) || _current_average_filter_a.getState() < FLT_EPSILON
	    || reset_filter_transition_now) {
		_current_average_filter_a.reset(_params.bat_avrg_current);
	}

	_vehicle_status_was_fw = _vehicle_status_is_fw;

	if (_armed && PX4_ISFINITE(current_a)) {
		// For FW only update when we are in level flight
		if (!_vehicle_status_is_fw || ((hrt_absolute_time() - _flight_phase_estimation_sub.get().timestamp) < 2_s
					       && _flight_phase_estimation_sub.get().flight_phase == flight_phase_estimation_s::FLIGHT_PHASE_LEVEL)) {
			if (_dt > FLT_EPSILON) {
				_current_average_filter_a.update(fmaxf(current_a, 0.f), _dt);

			} else {
				_current_average_filter_a.update(fmaxf(current_a, 0.f));
			}
		}
	}

	// Remaining time estimation only possible with capacity
	if (_capacity_mah > 0.f) {
		const float remaining_capacity_mah = _state_of_charge * _capacity_mah;
		const float current_ma = fmaxf(_current_average_filter_a.getState() * 1e3f, FLT_EPSILON);
		time_remaining_s = remaining_capacity_mah / current_ma * 3600.f;
	}

	return time_remaining_s;
}

void Battery::updateParams()
{
	// Apply profile/calibration changes while disarmed, never partway through a flight.
	if (_armed && !_first_parameter_update) {
		_parameter_update_pending = true;
		return;
	}

	_parameter_update_pending = false;
	const int n_cells = _first_parameter_update ? 0 : _params.n_cells;
	param_get(_param_handles.v_empty, &_params.v_empty);
	param_get(_param_handles.v_charged, &_params.v_charged);
	param_get(_param_handles.n_cells, &_params.n_cells);
	param_get(_param_handles.r_internal, &_params.r_internal);
	param_get(_param_handles.source, &_params.source);
	param_get(_param_handles.low_thr, &_params.low_thr);
	param_get(_param_handles.crit_thr, &_params.crit_thr);
	param_get(_param_handles.emergen_thr, &_params.emergen_thr);
	param_get(_param_handles.bat_avrg_current, &_params.bat_avrg_current);
	param_get(_param_handles.i_overwrite, &_params.i_overwrite);

	float capacity{0.f};
	param_get(_param_handles.capacity, &capacity);
	setCapacityMah(capacity);

	if (n_cells != _params.n_cells) {
		_internal_resistance_initialized = false;
	}

	if (!_internal_resistance_initialized && _params.n_cells > 0) {
		_RLS_est(0) = OCV_DEFAULT * _params.n_cells;
		_RLS_est(1) = R_DEFAULT * _params.n_cells;
		_estimation_covariance(0, 0) = OCV_COVARIANCE * _params.n_cells;
		_estimation_covariance(0, 1) = 0.f;
		_estimation_covariance(1, 0) = 0.f;
		_estimation_covariance(1, 1) = R_COVARIANCE * _params.n_cells;
		_estimation_covariance_norm = sqrtf(powf(_estimation_covariance(0, 0), 2.f) + 2.f * powf(_estimation_covariance(1, 0),
						    2.f) + powf(_estimation_covariance(1, 1), 2.f));
		_internal_resistance_initialized = true;
	}

	BatterySocEstimator::Config soc_config{};
	soc_config.capacity_mah = capacity;
	soc_config.cell_resistance = _params.r_internal;
	soc_config.current_overwrite = _params.i_overwrite > FLT_EPSILON;
	soc_config.cell_count = _params.n_cells;

	for (unsigned i = 0; i < BatterySocEstimator::PROFILE_POINTS; ++i) {
		param_get(_param_handles.soc_voltage[i], &soc_config.voltage[i]);
	}

	param_get(param_find("BAT_SOC_TC"), &soc_config.correction_time_constant);
	param_get(param_find("BAT_SOC_I_MAX"), &soc_config.current_max);
	param_get(param_find("BAT_SOC_I_STAB"), &soc_config.current_stability);
	param_get(param_find("BAT_SOC_SETTLE"), &soc_config.settling_time);
	param_get(param_find("BAT_SOC_F_SETT"), &soc_config.flight_settling_time);
	float power_span_percent{30.f};
	param_get(param_find("BAT_SOC_P_STAB"), &power_span_percent);
	soc_config.flight_power_stability = power_span_percent * 0.01f;
	param_get(param_find("BAT_SOC_F_DV"), &soc_config.flight_voltage_margin);
	float correction_percent_per_minute{2.f};
	param_get(param_find("BAT_SOC_F_RATE"), &correction_percent_per_minute);
	soc_config.flight_correction_rate = correction_percent_per_minute / 6000.f;
	int32_t in_flight{1};
	param_get(param_find("BAT_SOC_INFLT"), &in_flight);
	soc_config.correction_in_flight = in_flight != 0;
	param_get(_param_handles.voltage_low, &soc_config.voltage_low);
	param_get(_param_handles.voltage_critical, &soc_config.voltage_critical);
	param_get(_param_handles.voltage_emergency, &soc_config.voltage_emergency);
	param_get(param_find("BAT_V_TC"), &soc_config.voltage_filter_time_constant);
	param_get(param_find("BAT_V_DELAY"), &soc_config.voltage_delay);
	param_get(param_find("BAT_V_HYST"), &soc_config.voltage_hysteresis);
	int32_t mode{0};
	param_get(_param_handles.soc_mode, &mode);

	if (_soc_estimator.configure(soc_config) || mode == battery_soc_status_s::MODE_DISABLED) {
		_soc_active = false;
		_soc_estimator.reset();
	}

	if (mode != battery_soc_status_s::MODE_ACTIVE) {
		_soc_active = false;
	}

	_soc_mode = mode >= battery_soc_status_s::MODE_DISABLED && mode <= battery_soc_status_s::MODE_ACTIVE
		    ? mode : battery_soc_status_s::MODE_DISABLED;

	ModuleParams::updateParams();

	_first_parameter_update = false;
}
