// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#include "BatterySocEstimator.hpp"
#include <mathlib/mathlib.h>
#include <float.h>
#include <uORB/topics/battery_status.h>

bool BatterySocEstimator::configure(const Config &config)
{
	const auto different = [](float a, float b) {
		return !PX4_ISFINITE(a) || !PX4_ISFINITE(b) || fabsf(a - b) > FLT_EPSILON;
	};
	bool changed = different(config.cell_resistance, _config.cell_resistance)
		       || config.current_overwrite != _config.current_overwrite
		       || different(config.capacity_mah, _config.capacity_mah) || config.cell_count != _config.cell_count
		       || different(config.correction_time_constant, _config.correction_time_constant)
		       || different(config.current_max, _config.current_max) || different(config.current_stability, _config.current_stability)
		       || different(config.settling_time, _config.settling_time) || config.correction_in_flight != _config.correction_in_flight
		       || different(config.flight_settling_time, _config.flight_settling_time)
		       || different(config.flight_power_stability, _config.flight_power_stability)
		       || different(config.flight_voltage_margin, _config.flight_voltage_margin)
		       || different(config.flight_correction_rate, _config.flight_correction_rate)
		       || different(config.voltage_low, _config.voltage_low) || different(config.voltage_critical, _config.voltage_critical)
		       || different(config.voltage_emergency, _config.voltage_emergency)
		       || different(config.voltage_filter_time_constant, _config.voltage_filter_time_constant)
		       || different(config.voltage_delay, _config.voltage_delay) || different(config.voltage_hysteresis, _config.voltage_hysteresis);

	for (unsigned i = 0; i < PROFILE_POINTS; ++i) {
		changed |= different(config.voltage[i], _config.voltage[i]);
	}

	if (changed) {
		_config = config;
		reset();
	}

	return changed;
}

bool BatterySocEstimator::profileValid() const
{
	if (!PX4_ISFINITE(_config.capacity_mah) || _config.capacity_mah <= 0.f || _config.capacity_mah > 100000.f
	    || !PX4_ISFINITE(_config.cell_resistance) || _config.cell_resistance < -1.f || _config.cell_resistance > 0.2f
	    || _config.current_overwrite
	    || _config.cell_count < 1 || _config.cell_count > 16
	    || !PX4_ISFINITE(_config.correction_time_constant) || _config.correction_time_constant < 1.f
	    || !PX4_ISFINITE(_config.current_max) || _config.current_max <= 0.f
	    || !PX4_ISFINITE(_config.current_stability) || _config.current_stability <= 0.f
	    || !PX4_ISFINITE(_config.settling_time) || _config.settling_time < 1.f
	    || !PX4_ISFINITE(_config.flight_settling_time) || _config.flight_settling_time < 1.f
	    || !PX4_ISFINITE(_config.flight_power_stability) || _config.flight_power_stability <= 0.f
	    || _config.flight_power_stability > 1.f
	    || !PX4_ISFINITE(_config.flight_voltage_margin) || _config.flight_voltage_margin < 0.f
	    || _config.flight_voltage_margin > 0.2f
	    || !PX4_ISFINITE(_config.flight_correction_rate) || _config.flight_correction_rate <= 0.f) {
		return false;
	}

	for (unsigned i = 0; i < PROFILE_POINTS; ++i) {
		// Broad configuration bounds, not a chemistry-specific discharge curve.
		if (!PX4_ISFINITE(_config.voltage[i]) || _config.voltage[i] < 2.5f || _config.voltage[i] > 5.f
		    || (i > 0 && _config.voltage[i] <= _config.voltage[i - 1])) {
			return false;
		}
	}

	return true;
}

bool BatterySocEstimator::protectionValid() const
{
	return _config.cell_count > 0 && _config.cell_count <= 16
	       && PX4_ISFINITE(_config.voltage_emergency) && _config.voltage_emergency >= 2.5f
	       && PX4_ISFINITE(_config.voltage_critical) && _config.voltage_critical > _config.voltage_emergency
	       && PX4_ISFINITE(_config.voltage_low) && _config.voltage_low > _config.voltage_critical && _config.voltage_low <= 5.f
	       && PX4_ISFINITE(_config.voltage_filter_time_constant) && _config.voltage_filter_time_constant > 0.f
	       && PX4_ISFINITE(_config.voltage_delay) && _config.voltage_delay > 0.f
	       && PX4_ISFINITE(_config.voltage_hysteresis) && _config.voltage_hysteresis > 0.f;
}

float BatterySocEstimator::lookup(float cell_voltage) const
{
	if (!profileValid() || !PX4_ISFINITE(cell_voltage)) {
		return NAN;
	}

	if (cell_voltage <= _config.voltage[0]) {
		return 0.f;
	}

	for (unsigned i = 1; i < PROFILE_POINTS; ++i) {
		if (cell_voltage < _config.voltage[i]) {
			const float fraction = (cell_voltage - _config.voltage[i - 1]) / (_config.voltage[i] - _config.voltage[i - 1]);
			return (i - 1 + fraction) / (PROFILE_POINTS - 1);
		}
	}

	return 1.f;
}

void BatterySocEstimator::resetQualification()
{
	_qualifying = false;
	_stable_since = 0;
	_correction_eligible = false;
}

void BatterySocEstimator::reset()
{
	_last_timestamp = 0;
	_last_discharged_mah = NAN;
	_soc = _coulomb_soc = _voltage_soc = _terminal_voltage = NAN;
	_initialized = false;
	_charge_history_valid = false;
	_full_charge_confirmed = false;
	_voltage_warning = _warning_candidate = battery_status_s::WARNING_NONE;
	_warning_time = 0.f;
	_voltage_sample_valid = false;
	resetQualification();
}

float BatterySocEstimator::warningThreshold(uint8_t warning) const
{
	switch (warning) {
	case battery_status_s::WARNING_LOW: return _config.voltage_low;

	case battery_status_s::WARNING_CRITICAL: return _config.voltage_critical;

	case battery_status_s::WARNING_EMERGENCY: return _config.voltage_emergency;

	default: return 0.f;
	}
}

void BatterySocEstimator::updateVoltageProtection(float cell_voltage, float dt)
{
	if (!protectionValid() || !PX4_ISFINITE(cell_voltage) || cell_voltage <= 0.f) {
		_warning_time = 0.f; // Missing voltage cannot clear an existing warning.
		_voltage_sample_valid = false;
		return;
	}

	if (!_voltage_sample_valid) {
		dt = 0.f;
	}

	_voltage_sample_valid = true;

	if (!PX4_ISFINITE(_terminal_voltage) || dt <= 0.f) {
		_terminal_voltage = cell_voltage;
		_warning_time = 0.f;

	} else {
		const float alpha = 1.f - expf(-dt / _config.voltage_filter_time_constant);
		_terminal_voltage += alpha * (cell_voltage - _terminal_voltage);
	}

	uint8_t candidate = battery_status_s::WARNING_NONE;

	if (_terminal_voltage <= _config.voltage_emergency) {
		candidate = battery_status_s::WARNING_EMERGENCY;

	} else if (_terminal_voltage <= _config.voltage_critical) {
		candidate = battery_status_s::WARNING_CRITICAL;

	} else if (_terminal_voltage <= _config.voltage_low) {
		candidate = battery_status_s::WARNING_LOW;
	}

	if (candidate < _voltage_warning
	    && _terminal_voltage <= warningThreshold(_voltage_warning) + _config.voltage_hysteresis) {
		candidate = _voltage_warning;
	}

	if (candidate != _warning_candidate) {
		_warning_candidate = candidate;
		_warning_time = 0.f;

	} else {
		_warning_time += dt;
	}

	if (_warning_time >= _config.voltage_delay) {
		_voltage_warning = candidate;
	}
}

void BatterySocEstimator::update(uint64_t timestamp, bool connected, bool armed, bool vehicle_status_valid,
				 float current_a, float compensated_cell_voltage, float terminal_cell_voltage, float discharged_mah)
{
	if (!connected) {
		reset();
		return;
	}

	if (_last_timestamp != 0 && timestamp <= _last_timestamp) {
		return; // Duplicate/out-of-order samples cannot integrate charge or qualify relaxation.
	}

	const float elapsed = _last_timestamp != 0 ? (timestamp - _last_timestamp) * 1e-6f : 0.f;
	const bool continuous = elapsed > 0.f && elapsed <= MAX_SAMPLE_INTERVAL_S;
	_last_timestamp = timestamp;
	updateVoltageProtection(terminal_cell_voltage, continuous ? elapsed : 0.f);
	_voltage_soc = lookup(compensated_cell_voltage);
	_correction_eligible = false;

	if (!profileValid()) {
		resetQualification();
		_charge_history_valid = false;
		_last_discharged_mah = discharged_mah;
		return;
	}

	const bool current_valid = PX4_ISFINITE(current_a) && current_a >= 0.f
				   && PX4_ISFINITE(discharged_mah) && discharged_mah >= 0.f;
	const bool counter_valid = !PX4_ISFINITE(_last_discharged_mah) || discharged_mah >= _last_discharged_mah;
	const bool voltage_valid = PX4_ISFINITE(_voltage_soc) && PX4_ISFINITE(terminal_cell_voltage)
				   && terminal_cell_voltage > 0.f
				   && terminal_cell_voltage <= _config.voltage[PROFILE_POINTS - 1] * 1.05f;

	if (!_initialized && voltage_valid) {
		// As with the legacy estimator, startup SOC comes from the available voltage.
		// The calibrated lookup replaces linear interpolation; relaxation only gates corrections.
		_soc = _coulomb_soc = _voltage_soc;
		_initialized = true;
		_charge_history_valid = current_valid && counter_valid;

	} else if (_initialized && current_valid && counter_valid && PX4_ISFINITE(_last_discharged_mah)) {
		const float discharge = (discharged_mah - _last_discharged_mah) / _config.capacity_mah;
		_soc = math::constrain(_soc - discharge, 0.f, 1.f);
		_coulomb_soc = math::constrain(_coulomb_soc - discharge, 0.f, 1.f);
	}

	_last_discharged_mah = discharged_mah;

	if (!current_valid || !counter_valid || (!continuous && elapsed > 0.f)) {
		_charge_history_valid = false;
	}

	// Flight qualification uses measured electrical power, without the disarmed current limit.
	const float load = armed ? current_a * terminal_cell_voltage * _config.cell_count : current_a;
	const bool trustworthy = current_valid && counter_valid && voltage_valid && PX4_ISFINITE(load)
				 && vehicle_status_valid && (armed ? _config.correction_in_flight : current_a <= _config.current_max)
				 && (continuous || elapsed <= 0.f);

	if (!trustworthy) {
		resetQualification();
		return;
	}

	const float load_min = math::min(_load_min, load);
	const float load_max = math::max(_load_max, load);
	const float allowed_span = armed ? _config.flight_power_stability * (0.5f * load_min + 0.5f * load_max)
				   : _config.current_stability;

	if (!_qualifying || _qualifying_in_flight != armed || load_max - load_min > allowed_span) {
		_load_min = _load_max = load;
		_stable_since = timestamp;
		_qualifying = true;
		_qualifying_in_flight = armed;

	} else {
		_load_min = load_min;
		_load_max = load_max;
	}

	const float settling_time = armed ? _config.flight_settling_time : _config.settling_time;
	const float stable_time = (timestamp - _stable_since) * 1e-6f;
	_correction_eligible = stable_time >= settling_time;

	if (_correction_eligible) {
		_full_charge_confirmed |= !armed && _voltage_soc >= 1.f;

		if (!_charge_history_valid && !armed) {
			// Only a qualified disarmed rest can replace missing charge history. Loaded voltage
			// is less certain, so flight corrections remain gradual even after lost measurements.
			_soc = _coulomb_soc = _voltage_soc;
			_charge_history_valid = true;

		} else {
			// Only the part of this interval after qualification contributes to correction.
			const float correction_dt = math::min(elapsed, math::max(0.f, stable_time - settling_time));
			const float alpha = 1.f - expf(-correction_dt / _config.correction_time_constant);
			float target = _voltage_soc;

			if (armed) {
				// Treat loaded voltage as an interval, not an exact OCV measurement. Correct
				// toward the nearest edge only when SOC lies outside it, symmetrically.
				target = math::constrain(_soc, lookup(compensated_cell_voltage - _config.flight_voltage_margin),
							 lookup(compensated_cell_voltage + _config.flight_voltage_margin));
			}

			float correction = alpha * (target - _soc);

			if (armed) {
				const float limit = _config.flight_correction_rate * correction_dt;
				correction = math::constrain(correction, -limit, limit);
			}

			_soc = math::constrain(_soc + correction, 0.f, 1.f);
		}
	}
}
