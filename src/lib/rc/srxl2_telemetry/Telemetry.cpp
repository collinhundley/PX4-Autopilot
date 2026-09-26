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

#include "Telemetry.hpp"

#include <cmath>
#include <cstring>

namespace srxl2
{
namespace
{
constexpr double DegreesPerRadian = 57.29577951308232;
constexpr double MetresPerSecondToKnots = 1.9438444924406048;
constexpr uint64_t MicrosecondsPerSecond = 1000000;
constexpr uint64_t MicrosecondsPerDay = 86400ULL * MicrosecondsPerSecond;
constexpr uint8_t GpsNorth = 1 << 0;
constexpr uint8_t GpsEast = 1 << 1;
constexpr uint8_t GpsLongitudeOver99 = 1 << 2;
constexpr uint8_t GpsFixValid = 1 << 3;
constexpr uint8_t GpsDataReceived = 1 << 4;
constexpr uint8_t GpsFix3d = 1 << 5;
constexpr uint8_t GpsNegativeAltitude = 1 << 7;
}

bool Telemetry::fresh(uint64_t now, uint64_t stamp, uint64_t timeout)
{
	return stamp != 0 && now >= stamp && now - stamp <= timeout;
}

void Telemetry::put16(uint8_t *out, uint16_t value, bool big_endian)
{
	out[big_endian ? 1 : 0] = static_cast<uint8_t>(value);
	out[big_endian ? 0 : 1] = static_cast<uint8_t>(value >> 8);
}

void Telemetry::put32(uint8_t *out, uint32_t value)
{
	for (unsigned i = 0; i < sizeof(value); ++i) {
		out[i] = static_cast<uint8_t>(value >>(8 * i));
	}
}

uint16_t Telemetry::signedValue(double value)
{
	if (!std::isfinite(value)) {
		return SignedUnavailable;
	}

	// Reserve 0x7fff as unavailable; never wrap valid measurements.
	value = ::fmax(-32768., ::fmin(32766., value));
	return static_cast<uint16_t>(static_cast<int16_t>(::lround(value)));
}

uint16_t Telemetry::unsignedValue(double value)
{
	if (!std::isfinite(value) || value < 0) {
		return UnsignedUnavailable;
	}

	return static_cast<uint16_t>(::lround(::fmin(65534., value)));
}

uint32_t Telemetry::bcd(uint32_t value)
{
	uint32_t result = 0;

	for (unsigned shift = 0; shift < 32; shift += 4) {
		result |= (value % 10) << shift;
		value /= 10;
	}

	return result;
}

void Telemetry::initPayload(uint8_t *out, uint8_t identifier)
{
	std::memset(out, 0xff, PayloadSize);
	out[0] = identifier;
	out[1] = 0;
}

void Telemetry::setPosition(const Position &data)
{
	const bool continuous = data.vertical_valid && std::isfinite(data.local_altitude_m)
				&& data.timestamp != 0 && _position.vertical_valid
				&& data.reset_counter == _position.reset_counter
				&& data.timestamp > _position.timestamp
				&& data.timestamp - _position.timestamp <= HistoryMaxGap;

	if (!continuous) {
		_history_size = 0;
		_history_head = 0;
	}

	_position = data;

	if (!data.vertical_valid || !std::isfinite(data.local_altitude_m) || data.timestamp == 0) {
		return;
	}

	const size_t previous = (_history_head + HistorySize - 1) % HistorySize;

	if (_history_size == 0 || data.timestamp - _history[previous].timestamp >= HistoryInterval) {
		_history[_history_head] = {data.timestamp, data.local_altitude_m};
		_history_head = (_history_head + 1) % HistorySize;

		if (_history_size < HistorySize) {
			++_history_size;
		}
	}
}

void Telemetry::queueMessage(uint64_t timestamp, uint8_t severity, const char *text, size_t length)
{
	if (timestamp == 0 || text == nullptr || length == 0 || severity > 6) {
		return;
	}

	size_t target = QueueSize;
	size_t worst = 0;

	for (size_t i = 0; i < QueueSize; ++i) {
		if (_messages[i].timestamp == 0) {
			target = i;
			break;
		}

		if (_messages[i].severity > _messages[worst].severity
		    || (_messages[i].severity == _messages[worst].severity
			&& _messages[i].timestamp < _messages[worst].timestamp)) {
			worst = i;
		}
	}

	if (target == QueueSize) {
		++_dropped_messages;

		if (severity > _messages[worst].severity) {
			return;
		}

		target = worst;
	}

	Message &message = _messages[target];
	message = {};
	message.timestamp = timestamp;
	message.severity = severity;
	size_t count = 0;

	for (; count < length && count < MessageLength && text[count] != '\0'; ++count) {
		const unsigned char character = static_cast<unsigned char>(text[count]);
		message.text[count] = character >= ' ' && character <= '~' ? static_cast<char>(character) : ' ';
	}

	if (count == MessageLength && count < length && text[count] != '\0') {
		std::memcpy(message.text + MessageLength - 3, "...", 3);
	}
}

void Telemetry::encodeBattery(uint64_t now)
{
	uint8_t *rpm = _payloads[Rpm];
	uint8_t *capacity = _payloads[Capacity];
	initPayload(rpm, 0x7e);
	initPayload(capacity, 0x34);
	put16(rpm + 6, SignedUnavailable, true);
	rpm[8] = 0; // Receiver RSSI unavailable: let the receiver supply its own link data.
	rpm[9] = 0;
	put16(rpm + 14, 0, true); // No fastboot uptime data.

	for (size_t offset = 2; offset < 14; offset += 2) {
		put16(capacity + offset, SignedUnavailable, true);
	}

	if (fresh(now, _battery.timestamp, SensorTimeout) && _battery.connected) {
		_active[Capacity] = true;

		if (_battery.voltage_v > 0) {
			put16(rpm + 4, unsignedValue(static_cast<double>(_battery.voltage_v) * 100.), true);
		}

		if (_battery.current_a >= 0) {
			put16(capacity + 2, signedValue(static_cast<double>(_battery.current_a) * 10.), true);
		}

		if (_battery.discharged_mah >= 0) {
			put16(capacity + 4, signedValue(static_cast<double>(_battery.discharged_mah)), true);
		}

		if (_battery.temperature_valid && std::isfinite(_battery.temperature_c)) {
			put16(rpm + 6, signedValue(static_cast<double>(_battery.temperature_c) * 1.8 + 32.), true);

			if (_battery.temperature_c >= 0 && _battery.temperature_c <= 150) {
				put16(capacity + 6, unsignedValue(static_cast<double>(_battery.temperature_c) * 10.), true);
			}
		}
	}
}

void Telemetry::encodeGps(uint64_t now)
{
	uint8_t *location = _payloads[GpsLocation];
	initPayload(location, 0x16);
	initPayload(_gps_stats, 0x17);
	location[15] = 0;

	if (!fresh(now, _gps.timestamp, SensorTimeout)) {
		return;
	}

	_active[GpsLocation] = true;
	uint8_t flags = GpsDataReceived;
	const bool position_valid = _gps.fix_type >= 2 && std::isfinite(_gps.latitude_deg)
				    && std::isfinite(_gps.longitude_deg) && std::fabs(_gps.latitude_deg) <= 90
				    && std::fabs(_gps.longitude_deg) <= 180;

	if (position_valid) {
		flags |= GpsFixValid;
		flags |= _gps.latitude_deg >= 0 ? GpsNorth : 0;
		flags |= _gps.longitude_deg >= 0 ? GpsEast : 0;
		flags |= std::fabs(_gps.longitude_deg) >= 100 ? GpsLongitudeOver99 : 0;

		const double coordinates[2] = {_gps.latitude_deg, _gps.longitude_deg};

		for (size_t i = 0; i < 2; ++i) {
			const double absolute = std::fabs(coordinates[i]);
			unsigned degrees = static_cast<unsigned>(absolute);
			unsigned minutes = static_cast<unsigned>(::lround((absolute - degrees) * 600000.));

			if (minutes == 600000) {
				minutes = 0;
				++degrees;
			}

			if (i == 1 && degrees >= 100) {
				flags |= GpsLongitudeOver99;
				degrees -= 100;
			}

			put32(location + 4 + i * 4, bcd(degrees * 1000000 + minutes));
		}

		if (_gps.fix_type >= 3 && std::isfinite(_gps.altitude_msl_m)) {
			flags |= GpsFix3d;
			flags |= _gps.altitude_msl_m < 0 ? GpsNegativeAltitude : 0;
			const uint32_t altitude = static_cast<uint32_t>(::lround(::fmin(999999.,
						  std::fabs(_gps.altitude_msl_m) * 10.)));
			put16(location + 2, static_cast<uint16_t>(bcd(altitude % 10000)));
			_gps_stats[9] = static_cast<uint8_t>(bcd(altitude / 10000));
		}

		if (_gps.velocity_valid && std::isfinite(_gps.course_rad)) {
			double course = ::fmod(static_cast<double>(_gps.course_rad) * DegreesPerRadian, 360.);

			if (course < 0) {
				course += 360.;
			}

			put16(location + 12, static_cast<uint16_t>(bcd(static_cast<uint32_t>(::lround(course * 10.)) % 3600)));
		}

		if (_gps.velocity_valid && std::isfinite(_gps.speed_m_s) && _gps.speed_m_s >= 0) {
			const uint32_t speed = static_cast<uint32_t>(::lround(::fmin(9999.,
					       static_cast<double>(_gps.speed_m_s) * MetresPerSecondToKnots * 10.)));
			put16(_gps_stats + 2, static_cast<uint16_t>(bcd(speed)));
		}
	}

	if (std::isfinite(_gps.hdop) && _gps.hdop >= 0) {
		location[14] = static_cast<uint8_t>(bcd(static_cast<uint32_t>(::lround(::fmin(99., static_cast<double>(_gps.hdop) * 10.)))));
	}

	_gps_stats[8] = static_cast<uint8_t>(bcd(_gps.satellites > 99 ? 99 : _gps.satellites));

	if (_gps.time_utc_usec != 0) {
		const uint32_t tenths = (_gps.time_utc_usec % MicrosecondsPerDay) / 100000;
		const uint32_t seconds = tenths / 10;
		const uint32_t decimal_time = (seconds / 3600) * 100000 + ((seconds / 60) % 60) * 1000
					      + (seconds % 60) * 10 + tenths % 10;
		put32(_gps_stats + 4, bcd(decimal_time));
	}

	location[15] = flags;
}

uint16_t Telemetry::climbRate(uint64_t window) const
{
	if (_position.timestamp < window || _history_size == 0) {
		return SignedUnavailable;
	}

	const uint64_t target = _position.timestamp - window;
	HistorySample lower{};

	for (size_t i = 0; i < _history_size; ++i) {
		const HistorySample &sample = _history[(_history_head + HistorySize - _history_size + i) % HistorySize];

		if (sample.timestamp == target) {
			return signedValue(static_cast<double>(_position.local_altitude_m - sample.altitude_m) * 10. * MicrosecondsPerSecond / window);
		}

		if (sample.timestamp > target) {
			if (lower.timestamp == 0) {
				return SignedUnavailable;
			}

			const double fraction = static_cast<double>(target - lower.timestamp) / (sample.timestamp - lower.timestamp);
			const double altitude = static_cast<double>(lower.altitude_m) + fraction * static_cast<double>(sample.altitude_m - lower.altitude_m);
			return signedValue((static_cast<double>(_position.local_altitude_m) - altitude) * 10. * MicrosecondsPerSecond / window);
		}

		lower = sample;
	}

	return SignedUnavailable;
}

void Telemetry::encodeVario(uint64_t now)
{
	uint8_t *out = _payloads[Vario];
	initPayload(out, 0x40);

	for (size_t offset = 2; offset < PayloadSize; offset += 2) {
		put16(out + offset, SignedUnavailable, true);
	}

	if (!fresh(now, _position.timestamp, MotionTimeout)) {
		return;
	}

	if (_position.altitude_valid && std::isfinite(_position.altitude_m)) {
		_active[Vario] = true;
		put16(out + 2, signedValue(static_cast<double>(_position.altitude_m) * 10.), true);
	}

	if (_position.vertical_valid && std::isfinite(_position.local_altitude_m)) {
		_active[Vario] = true;
		static constexpr uint64_t windows[] = {250000, 500000, 1000000, 1500000, 2000000, 3000000};

		for (size_t i = 0; i < sizeof(windows) / sizeof(windows[0]); ++i) {
			put16(out + 4 + i * 2, climbRate(windows[i]), true);
		}
	}
}

void Telemetry::encodeAttitude(uint64_t now)
{
	uint8_t *out = _payloads[AttitudeMag];
	initPayload(out, 0x1b);

	for (size_t offset = 2; offset < 14; offset += 2) {
		put16(out + offset, SignedUnavailable, true);
	}

	if (fresh(now, _attitude.timestamp, MotionTimeout) && _attitude.valid) {
		_active[AttitudeMag] = true;
		put16(out + 2, signedValue(static_cast<double>(_attitude.roll_rad) * DegreesPerRadian * 10.), true);
		put16(out + 4, signedValue(static_cast<double>(_attitude.pitch_rad) * DegreesPerRadian * 10.), true);
		put16(out + 6, signedValue(static_cast<double>(_attitude.yaw_rad) * DegreesPerRadian * 10.), true);
		// Magnetic vector and heading remain unavailable. Estimated yaw is not a magnetic heading.
	}
}

void Telemetry::encodeText(uint64_t now)
{
	for (size_t row = 0; row < TextRows; ++row) {
		std::memset(_text[row], 0, PayloadSize);
		_text[row][0] = 0x0c;
		_text[row][2] = row;
	}

	std::memcpy(_text[0] + 3, "PX4", 3);
	const char *mode = "Unknown mode";
	const char *arming = "State unknown";
	const char *status = "No status";

	if (fresh(now, _status.timestamp, MotionTimeout)) {
		_active[Text] = true;
		mode = _status.mode;
		arming = _status.armed ? "Armed" : "Disarmed";
		status = _status.failsafe ? "FAILSAFE" : (_status.ready ? "Ready" : "Not ready");
	}

	const char *strings[] = {mode, arming, status};

	for (size_t row = 1; row <= 3; ++row) {
		for (size_t column = 0; column < TextColumns && strings[row - 1][column] != '\0'; ++column) {
			_text[row][3 + column] = strings[row - 1][column];
		}
	}

	const Message *selected = nullptr;

	for (Message &message : _messages) {
		if (!fresh(now, message.timestamp, MessageTimeout)) {
			message.timestamp = 0;
			continue;
		}

		if (selected == nullptr || message.severity < selected->severity
		    || (message.severity == selected->severity && message.timestamp > selected->timestamp)) {
			selected = &message;
		}
	}

	if (selected != nullptr) {
		_active[Text] = true;
		size_t offset = 0;

		for (size_t row = 4; row < TextRows; ++row) {
			for (size_t column = 0; column < TextColumns && selected->text[offset] != '\0'; ++column) {
				_text[row][3 + column] = selected->text[offset++];
			}
		}
	}
}

void Telemetry::prepare(uint64_t now)
{
	_active[Rpm] = true;
	_active[Qos] = true;
	initPayload(_payloads[Qos], 0x7f); // Receiver fills its actual QoS. Never fabricate fades, holds or receiver voltage.
	encodeBattery(now);
	encodeGps(now);
	encodeVario(now);
	encodeAttitude(now);
	encodeText(now);
}

bool Telemetry::nextPayload(uint64_t now, uint8_t payload[PayloadSize])
{
	if (_gps_pending) {
		_gps_pending = false;

		if (now <= _gps_pair_expires) {
			std::memcpy(payload, _pending_gps_stats, PayloadSize);
			return true;
		}
	}

	// QoS/RPM 10 Hz, capacity and GPS pairs 2 Hz, vario/attitude 5 Hz, text rows 10 Hz.
	static constexpr uint64_t intervals[SlotCount] = {100000, 500000, 500000, 200000, 200000, 100000, 100000};

	for (size_t i = 0; i < SlotCount; ++i) {
		const uint8_t slot = (_next_slot + i) % SlotCount;

		if (!_active[slot] || (_sent[slot] && now >= _last_sent[slot] && now - _last_sent[slot] < intervals[slot])) {
			continue;
		}

		_sent[slot] = true;
		_last_sent[slot] = now;
		_next_slot = (slot + 1) % SlotCount;

		if (slot == Text) {
			std::memcpy(payload, _text[_next_text_row], PayloadSize);
			_next_text_row = (_next_text_row + 1) % TextRows;

		} else {
			std::memcpy(payload, _payloads[slot], PayloadSize);
		}

		if (slot == GpsLocation) {
			std::memcpy(_pending_gps_stats, _gps_stats, PayloadSize);
			_gps_pending = true;
			_gps_pair_expires = fresh(now, _gps.timestamp, SensorTimeout) ? _gps.timestamp + SensorTimeout : now + SensorTimeout;
		}

		return true;
	}

	return false;
}

} // namespace srxl2
