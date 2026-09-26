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

#include "Srxl2Telemetry.hpp"

#include <lib/modes/ui.hpp>
#include <matrix/math.hpp>

#include <cmath>
#include <cstring>

void Srxl2Telemetry::update(hrt_abstime now)
{
	// Reuse stack storage between topics: this runs on the small serial work-queue stack.
	{
		battery_status_s battery{};

		// Instance zero is the primary flight battery, as in the existing RC telemetry drivers.
		if (_battery_sub.update(&battery)) {
			srxl2::Telemetry::Battery sample{};
			sample.timestamp = battery.timestamp;
			sample.connected = battery.connected;
			sample.voltage_v = battery.voltage_v;
			sample.current_a = battery.current_a;
			sample.discharged_mah = battery.discharged_mah;
			sample.temperature_c = battery.temperature;
			sample.temperature_valid = std::isfinite(battery.temperature);
			_telemetry.setBattery(sample);
		}
	}

	{
		sensor_gps_s gps{};

		if (_gps_sub.update(&gps)) {
			srxl2::Telemetry::Gps sample{};
			sample.timestamp = gps.timestamp;
			sample.time_utc_usec = gps.time_utc_usec;
			sample.latitude_deg = gps.latitude_deg;
			sample.longitude_deg = gps.longitude_deg;
			sample.altitude_msl_m = gps.altitude_msl_m;
			sample.speed_m_s = gps.vel_m_s;
			sample.course_rad = gps.cog_rad;
			sample.hdop = gps.hdop;
			sample.fix_type = gps.fix_type;
			sample.satellites = gps.satellites_used;
			sample.velocity_valid = gps.vel_ned_valid;
			_telemetry.setGps(sample);
		}
	}

	{
		home_position_s home{};

		if (_home_sub.update(&home)) {
			_home_z = home.z;
			_home_valid = home.valid_alt && home.valid_lpos && std::isfinite(home.z);
		}
	}

	{
		vehicle_local_position_s position{};

		if (_position_sub.update(&position)) {
			srxl2::Telemetry::Position sample{};
			sample.timestamp = position.timestamp;
			sample.local_altitude_m = -position.z;
			sample.altitude_m = _home_z - position.z;
			sample.altitude_valid = position.z_valid && _home_valid;
			sample.vertical_valid = position.z_valid && position.v_z_valid;
			sample.reset_counter = position.z_reset_counter;
			_telemetry.setPosition(sample);
		}
	}

	{
		vehicle_attitude_s attitude{};

		if (_attitude_sub.update(&attitude)) {
			srxl2::Telemetry::Attitude sample{};
			sample.timestamp = attitude.timestamp;
			const matrix::Quatf quaternion(attitude.q);
			const float norm = quaternion.norm();
			sample.valid = std::isfinite(norm) && std::fabs(norm - 1.f) < 0.01f;

			if (sample.valid) {
				const matrix::Eulerf euler(quaternion);
				sample.roll_rad = euler.phi();
				sample.pitch_rad = euler.theta();
				sample.yaw_rad = euler.psi();
			}

			_telemetry.setAttitude(sample);
		}
	}

	{
		vehicle_status_s status{};

		if (_status_sub.update(&status)) {
			srxl2::Telemetry::Status sample{};
			sample.timestamp = status.timestamp;
			sample.armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
			sample.failsafe = status.failsafe;
			sample.ready = status.pre_flight_checks_pass;
			const char *name = status.nav_state_display < vehicle_status_s::NAVIGATION_STATE_MAX ?
					   mode_util::nav_state_names[status.nav_state_display] : "Unknown mode";
			std::strncpy(sample.mode, name, sizeof(sample.mode) - 1);
			_telemetry.setStatus(sample);
		}
	}

	// Bounded work: drain at most the topic queue once per update, never during a bus reply.
	for (unsigned i = 0; i < mavlink_log_s::ORB_QUEUE_LENGTH && _log_sub.updated(); ++i) {
		const unsigned previous_generation = _log_sub.get_last_generation();
		mavlink_log_s message{};

		if (_log_sub.update(&message)) {
			const unsigned delta = _log_sub.get_last_generation() - previous_generation;

			if (previous_generation != 0 && delta > 1) {
				_missed_log_messages += delta - 1;
			}

			_telemetry.queueMessage(message.timestamp, message.severity, message.text, sizeof(message.text));
		}
	}

	_telemetry.prepare(now);
}
