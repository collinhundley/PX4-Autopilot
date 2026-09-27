// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#include "OsdTelemetry.hpp"

#include <lib/geo/geo.h>
#include <lib/modes/ui.hpp>
#include <matrix/matrix/math.hpp>
#include <cstdio>
#include <cstring>

namespace msp_osd
{

bool OsdTelemetryCore::fresh(uint64_t now, uint64_t timestamp, uint64_t maximum_age)
{
	return timestamp > 0 && timestamp <= now && now - timestamp <= maximum_age;
}

bool OsdTelemetryCore::coordinatesValid(double lat, double lon)
{
	return std::isfinite(lat) && std::isfinite(lon) && fabs(lat) <= 90.0 && fabs(lon) <= 180.0;
}

float OsdTelemetryCore::thrustMagnitude(uint64_t now, const vehicle_thrust_setpoint_s &thrust)
{
	if (!fresh(now, thrust.timestamp, SECOND_US / 2)) {
		return NAN;
	}

	float magnitude_squared = 0.f;

	for (float component : thrust.xyz) {
		// VehicleThrustSetpoint defines NaN as motors stopped on this axis.
		// Standard VTOL uses it for the idle pusher while vertical thrust remains valid.
		if (std::isnan(component)) { continue; }

		if (!std::isfinite(component)) { return NAN; }

		const float bounded = fminf(fabsf(component), 1.f);
		magnitude_squared += bounded * bounded;
	}

	return fminf(sqrtf(magnitude_squared), 1.f);
}

void OsdTelemetryCore::update(uint64_t now, const Samples &s, const Settings &settings)
{
	// Start with unavailable values; validity never leaks from an earlier sample.
	_data = OsdData{};

	if (fresh(now, s.battery.timestamp, 3 * SECOND_US) && s.battery.connected) {
		if (std::isfinite(s.battery.voltage_v) && s.battery.voltage_v > 0.f) {
			_data.battery_voltage = s.battery.voltage_v;

			if (s.battery.cell_count > 0) {
				_data.cell_voltage = s.battery.voltage_v / s.battery.cell_count;
			}
		}

		if (std::isfinite(s.battery.current_a) && s.battery.current_a >= 0.f) {
			_data.current_a = s.battery.current_a;
		}

		if (std::isfinite(s.battery.voltage_v_compensated) && s.battery.voltage_v_compensated > 0.f) {
			_data.compensated_battery_voltage = s.battery.voltage_v_compensated;

			if (s.battery.cell_count > 0) {
				_data.compensated_cell_voltage = s.battery.voltage_v_compensated / s.battery.cell_count;
			}
		}

		// Use the battery source's existing state of charge, including partial-charge
		// initialization. Do not recompute it from consumed mAh or OCV in the OSD.
		if (std::isfinite(s.battery.remaining) && s.battery.remaining >= 0.f && s.battery.remaining <= 1.f) {
			_data.battery_remaining_percent = s.battery.remaining * 100.f;
		}

		if (std::isfinite(s.battery.discharged_mah) && s.battery.discharged_mah >= 0.f) {
			_data.discharged_mah = s.battery.discharged_mah;
		}
	}

	if (fresh(now, s.airspeed.timestamp, SECOND_US)
	    && s.airspeed.airspeed_source >= airspeed_validated_s::SOURCE_GROUND_MINUS_WIND
	    && s.airspeed.airspeed_source <= airspeed_validated_s::SOURCE_SYNTHETIC
	    && std::isfinite(s.airspeed.indicated_airspeed_m_s) && s.airspeed.indicated_airspeed_m_s >= 0.f) {
		_data.airspeed_m_s = s.airspeed.indicated_airspeed_m_s;
		_data.airspeed_estimated = s.airspeed.airspeed_source == airspeed_validated_s::SOURCE_GROUND_MINUS_WIND
					   || s.airspeed.airspeed_source == airspeed_validated_s::SOURCE_SYNTHETIC;
	}

	const bool home_known = s.home.timestamp > 0 && s.home.timestamp <= now;

	if (fresh(now, s.position.timestamp, SECOND_US)) {
		if (s.position.v_xy_valid && std::isfinite(s.position.vx) && std::isfinite(s.position.vy)) {
			_data.ground_speed_m_s = hypotf(s.position.vx, s.position.vy);
		}

		// Home is persistent state, not a continuously sampled sensor. Local Z only
		// needs valid_lpos; valid_alt describes the independent AMSL home altitude.
		if (s.position.z_valid && home_known && s.home.valid_lpos
		    && std::isfinite(s.position.z) && std::isfinite(s.home.z)) {
			_data.altitude_m = s.home.z - s.position.z;
		}

		if (s.position.v_z_valid && std::isfinite(s.position.vz)) {
			_data.vertical_speed_m_s = -s.position.vz;
		}
	}

	if (fresh(now, s.global.timestamp, SECOND_US) && s.global.lat_lon_valid
	    && coordinatesValid(s.global.lat, s.global.lon) && home_known && s.home.valid_hpos
	    && coordinatesValid(s.home.lat, s.home.lon)) {
		_data.home_distance_m = get_distance_to_next_waypoint(s.global.lat, s.global.lon, s.home.lat, s.home.lon);

		// At home the direction is undefined; still display the valid zero distance.
		if (_data.home_distance_m >= 1.f) {
			_data.home_bearing_rad = get_bearing_to_next_waypoint(s.global.lat, s.global.lon, s.home.lat, s.home.lon);
		}
	}

	if (fresh(now, s.attitude.timestamp, SECOND_US / 2)) {
		matrix::Quatf attitude(s.attitude.q);
		const float norm_squared = attitude.norm_squared();

		// Reject corrupt quaternions, allowing normal estimator round-off before normalization.
		if (std::isfinite(norm_squared) && fabsf(norm_squared - 1.f) <= 0.1f) {
			attitude.normalize();
			attitude.copyTo(_data.attitude_q);
			const matrix::Eulerf euler(attitude);
			_data.roll_rad = euler.phi();
			_data.pitch_rad = euler.theta();
		}
	}

	if (fresh(now, s.gnss.timestamp, 2 * SECOND_US)) {
		_data.satellites = s.gnss.satellites_used;

		if (s.gnss.fix_type >= sensor_gps_s::FIX_TYPE_2D && coordinatesValid(s.gnss.latitude_deg, s.gnss.longitude_deg)) {
			_data.latitude_deg = s.gnss.latitude_deg;
			_data.longitude_deg = s.gnss.longitude_deg;
		}
	}

	if (fresh(now, s.rc.timestamp_last_signal, SECOND_US / 2) && !s.rc.rc_lost && !s.rc.rc_failsafe
	    && s.rc.rssi >= 0 && s.rc.rssi <= input_rc_s::RSSI_MAX) {
		_data.rssi_percent = s.rc.rssi;
	}

	_data.status_valid = fresh(now, s.status.timestamp, 2 * SECOND_US);

	if (_data.status_valid) {
		_data.armed = s.status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		_data.failsafe = s.status.failsafe;
		const char *name = s.status.nav_state_display < vehicle_status_s::NAVIGATION_STATE_MAX ?
				   mode_util::nav_state_names[s.status.nav_state_display] : "Unknown";
		const char *vehicle = "";

		if (s.status.is_vtol) {
			vehicle = s.status.in_transition_mode ? (s.status.in_transition_to_fw ? " >FW" : " >MC") :
				  (s.status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING ? " FW" : " MC");
		}

		snprintf(_data.mode, sizeof(_data.mode), "%.18s%s", name, vehicle);

		if (!_data.armed) {
			_data.throttle_percent = 0.f;

		} else {
			// Same operator-feedback convention as MAVLink VFR_HUD, with stale instances excluded.
			const float thrust_0 = thrustMagnitude(now, s.thrust[0]);
			const float thrust_1 = thrustMagnitude(now, s.thrust[1]);
			_data.throttle_percent = 100.f * fmaxf(thrust_0, thrust_1);
		}

		updateTimer(now, s.status, s.land);
		_data.flight_time_valid = true;
	}

	_data.flight_seconds = _flight_seconds;
	updateMessages(now, settings);
}

void OsdTelemetryCore::updateTimer(uint64_t now, const vehicle_status_s &status, const vehicle_land_detected_s &land)
{
	const bool landed = fresh(now, land.timestamp, SECOND_US) && land.landed;

	if (_data.armed && !landed && status.takeoff_time > 0 && status.takeoff_time <= now
	    && status.takeoff_time != _takeoff_time) {
		_takeoff_time = status.takeoff_time;
		_flight_running = true;
	}

	if (_flight_running) {
		uint64_t end = now;

		if (landed || !_data.armed) {
			end = landed ? land.timestamp : status.timestamp;
			_flight_running = false;
		}

		if (end >= _takeoff_time) {
			const uint64_t elapsed = (end - _takeoff_time) / SECOND_US;
			_flight_seconds = elapsed < UINT32_MAX ? elapsed : UINT32_MAX;
		}
	}
}

void OsdTelemetryCore::pushMessage(uint64_t now, const mavlink_log_s &message, const Settings &settings)
{
	const uint64_t duration = static_cast<uint64_t>(settings.message_duration_ms) * 1000;

	if (settings.log_level < 0 || settings.log_level >= 8 || message.severity > settings.log_level
	    || message.severity > 7 || !fresh(now, message.timestamp, duration) || message.text[0] == '\0') {
		return;
	}

	Message incoming{};
	incoming.timestamp = message.timestamp;
	incoming.expires = message.timestamp + duration;
	incoming.severity = message.severity;

	while (incoming.length < MESSAGE_LENGTH && message.text[incoming.length] != '\0') {
		const unsigned char c = message.text[incoming.length];
		incoming.text[incoming.length++] = c >= ' ' && c <= '~' ? c : ' ';
	}

	// Publishers timestamp before publishing, so publication order need not match timestamp order.
	// uORB generations consume each sample once; only suppress an exact queued duplicate here.
	for (const Message &entry : _messages) {
		if (entry.timestamp == incoming.timestamp && entry.severity == incoming.severity
		    && entry.length == incoming.length && memcmp(entry.text, incoming.text, incoming.length) == 0) {
			return;
		}
	}

	unsigned slot = MESSAGE_SLOTS;

	for (unsigned i = 0; i < MESSAGE_SLOTS; ++i) {
		if (_messages[i].expires <= now) {
			slot = i;
			break;
		}

		// A full queue replaces the oldest entry at its least important severity.
		if (slot == MESSAGE_SLOTS || _messages[i].severity > _messages[slot].severity
		    || (_messages[i].severity == _messages[slot].severity && _messages[i].timestamp < _messages[slot].timestamp)) {
			slot = i;
		}
	}

	if (_messages[slot].expires > now && _messages[slot].severity < message.severity) {
		return;
	}

	incoming.id = ++_last_message_id;

	if (incoming.id == 0) {
		incoming.id = ++_last_message_id;
	}

	_messages[slot] = incoming;
}

void OsdTelemetryCore::updateMessages(uint64_t now, const Settings &settings)
{
	Message *selected = nullptr;

	for (Message &message : _messages) {
		if (message.expires <= now || settings.log_level < 0 || settings.log_level >= 8
		    || message.severity > settings.log_level) {
			message.expires = 0;
			continue;
		}

		if (selected == nullptr || message.severity < selected->severity
		    || (message.severity == selected->severity && message.timestamp < selected->timestamp)) {
			selected = &message;
		}
	}

	if (selected == nullptr) {
		_shown_message_id = 0;
		return;
	}

	if (selected->id != _shown_message_id) {
		_shown_message_id = selected->id;
		_shown_since = now;
	}

	unsigned offset = 0;

	if (selected->length > DISPLAY_LENGTH && settings.scroll_ms > 0) {
		const unsigned final_offset = selected->length - DISPLAY_LENGTH;
		const uint64_t scroll_us = static_cast<uint64_t>(settings.scroll_ms) * 1000;
		const uint64_t dwell_us = static_cast<uint64_t>(settings.dwell_ms) * 1000;
		const uint64_t cycle_us = 2 * dwell_us + final_offset * scroll_us;
		const uint64_t phase = (now - _shown_since) % cycle_us;

		if (phase > dwell_us) {
			const uint64_t requested_offset = (phase - dwell_us) / scroll_us;
			offset = requested_offset < final_offset ? requested_offset : final_offset;
		}
	}

	const unsigned remaining = selected->length - offset;
	const unsigned count = remaining < DISPLAY_LENGTH ? remaining : DISPLAY_LENGTH;
	memcpy(_data.message, selected->text + offset, count);
	_data.message[count] = '\0';
}

void OsdTelemetry::update(uint64_t now, const Settings &settings)
{
	_battery_sub.update(&_samples.battery);
	_airspeed_sub.update(&_samples.airspeed);
	_position_sub.update(&_samples.position);
	_global_sub.update(&_samples.global);
	_home_sub.update(&_samples.home);
	_attitude_sub.update(&_samples.attitude);
	_status_sub.update(&_samples.status);
	_land_sub.update(&_samples.land);
	_thrust_0_sub.update(&_samples.thrust[0]);
	_thrust_1_sub.update(&_samples.thrust[1]);
	_gnss_sub.update(&_samples.gnss);
	_rc_sub.update(&_samples.rc);

	// Drain no more than one topic queue per render tick, even under a log flood.
	for (unsigned i = 0; i < mavlink_log_s::ORB_QUEUE_LENGTH && _log_sub.updated(); ++i) {
		mavlink_log_s message{};

		if (_log_sub.update(&message)) {
			_core.pushMessage(now, message, settings);
		}
	}

	_core.update(now, _samples, settings);
}

} // namespace msp_osd
