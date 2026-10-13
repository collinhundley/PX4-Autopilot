// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#pragma once

#include <stdint.h>
#include <math.h>

/** Coulomb counting with a calibrated, relaxed-voltage profile. No persistent charge history. */
class BatterySocEstimator
{
public:
	static constexpr unsigned PROFILE_POINTS = 41; // Voltages at 0, 2.5, ... 100 percent SOC.
	static constexpr float MAX_SAMPLE_INTERVAL_S = 5.f;

	struct Config {
		float voltage[PROFILE_POINTS] {};
		float capacity_mah{0.f};
		float cell_resistance{-1.f};
		bool current_overwrite{false};
		int32_t cell_count{0};
		float correction_time_constant{300.f};
		float current_max{2.f};
		float current_stability{0.5f};
		float settling_time{120.f};
		bool correction_in_flight{true};
		float flight_settling_time{10.f};
		float flight_power_stability{0.3f}; // Full power span relative to its midpoint.
		float flight_voltage_margin{0.02f}; // Per-cell uncertainty in compensated loaded voltage.
		float flight_correction_rate{0.02f / 60.f}; // SOC fraction per second, in either direction.
		float voltage_low{3.5f};
		float voltage_critical{3.4f};
		float voltage_emergency{3.3f};
		float voltage_filter_time_constant{1.f};
		float voltage_delay{2.f};
		float voltage_hysteresis{0.05f};
	};

	// Returns true if configuration changed and charge history was invalidated.
	bool configure(const Config &config);
	void reset();
	void update(uint64_t timestamp, bool connected, bool armed, bool vehicle_status_valid,
		    float current_a, float compensated_cell_voltage, float terminal_cell_voltage, float discharged_mah);
	bool profileValid() const;
	bool protectionValid() const;
	float lookup(float cell_voltage) const;
	float soc() const { return _soc; }
	float coulombSoc() const { return _coulomb_soc; }
	float voltageSoc() const { return _voltage_soc; }
	float filteredTerminalVoltage() const { return _terminal_voltage; }
	bool initialized() const { return _initialized; }
	bool chargeHistoryValid() const { return _charge_history_valid; }
	bool correctionEligible() const { return _correction_eligible; }
	bool fullChargeConfirmed() const { return _full_charge_confirmed; }
	uint8_t voltageWarning() const { return _voltage_warning; }

private:
	void resetQualification();
	void updateVoltageProtection(float cell_voltage, float dt);
	float warningThreshold(uint8_t warning) const;

	Config _config{};
	uint64_t _last_timestamp{0};
	float _last_discharged_mah{NAN};
	float _soc{NAN};
	float _coulomb_soc{NAN};
	float _voltage_soc{NAN};
	float _terminal_voltage{NAN};
	float _load_min{0.f};
	float _load_max{0.f};
	uint64_t _stable_since{0};
	float _warning_time{0.f};
	uint8_t _voltage_warning{0};
	uint8_t _warning_candidate{0};
	bool _voltage_sample_valid{false};
	bool _qualifying{false};
	bool _qualifying_in_flight{false};
	bool _initialized{false};
	bool _charge_history_valid{false};
	bool _correction_eligible{false};
	bool _full_charge_confirmed{false};
};
