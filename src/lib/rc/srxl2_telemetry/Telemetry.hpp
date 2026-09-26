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

#pragma once

#include <cstddef>
#include <cstdint>

namespace srxl2
{

/** Native Spektrum telemetry, independent of uORB and UART transport.
 * Wire formats: SpektrumRC/SpektrumDocumentation, public-domain
 * Telemetry/spektrumTelemetrySensors.h at 4e179359e0916f347b214f7825640733f54d7996.
 * Packed-BCD GPS fields use little endian. Numeric sensor fields use big endian.
 */
class Telemetry
{
public:
	static constexpr size_t PayloadSize = 16;
	static constexpr size_t TextColumns = 13;
	static constexpr size_t TextRows = 9;
	static constexpr size_t MessageLength = 5 * TextColumns;
	static constexpr uint64_t SensorTimeout = 2000000;
	static constexpr uint64_t MotionTimeout = 1000000;
	static constexpr uint64_t MessageTimeout = 5000000;

	struct Battery {
		uint64_t timestamp{0};
		bool connected{false};
		float voltage_v{0};
		float current_a{-1};
		float discharged_mah{-1};
		float temperature_c{0};
		bool temperature_valid{false};
	};

	struct Gps {
		uint64_t timestamp{0};
		uint64_t time_utc_usec{0};
		double latitude_deg{0};
		double longitude_deg{0};
		double altitude_msl_m{0};
		float speed_m_s{0};
		float course_rad{0};
		float hdop{0};
		uint8_t fix_type{0};
		uint8_t satellites{0};
		bool velocity_valid{false};
	};

	struct Position {
		uint64_t timestamp{0};
		float altitude_m{0}; ///< Relative to home, positive up.
		float local_altitude_m{0}; ///< -local_position.z, before home subtraction.
		bool altitude_valid{false};
		bool vertical_valid{false};
		uint8_t reset_counter{0};
	};

	struct Attitude {
		uint64_t timestamp{0};
		float roll_rad{0};
		float pitch_rad{0};
		float yaw_rad{0};
		bool valid{false};
	};

	struct Status {
		uint64_t timestamp{0};
		char mode[TextColumns + 1] {};
		bool armed{false};
		bool failsafe{false};
		bool ready{false};
	};

	void setBattery(const Battery &data) { _battery = data; }
	void setGps(const Gps &data) { _gps = data; }
	void setPosition(const Position &data);
	void setAttitude(const Attitude &data) { _attitude = data; }
	void setStatus(const Status &data) { _status = data; }
	void queueMessage(uint64_t timestamp, uint8_t severity, const char *text, size_t length);

	/** Prepare and expire cached frames outside the response-critical UART path. */
	void prepare(uint64_t now);
	/** Select one due cached frame. No allocation, IO, or sensor conversion. */
	bool nextPayload(uint64_t now, uint8_t payload[PayloadSize]);
	uint32_t droppedMessages() const { return _dropped_messages; }

private:
	enum Slot : uint8_t { Rpm, Capacity, GpsLocation, Vario, AttitudeMag, Text, Qos, SlotCount };
	static constexpr size_t HistorySize = 64;
	static constexpr uint64_t HistoryInterval = 50000;
	static constexpr uint64_t HistoryMaxGap = 250000;
	static constexpr size_t QueueSize = 8;
	static constexpr uint16_t SignedUnavailable = 0x7fff;
	static constexpr uint16_t UnsignedUnavailable = 0xffff;

	struct HistorySample {
		uint64_t timestamp{0};
		float altitude_m{0};
	};

	struct Message {
		uint64_t timestamp{0};
		uint8_t severity{0};
		char text[MessageLength + 1] {};
	};

	static bool fresh(uint64_t now, uint64_t stamp, uint64_t timeout);
	static void put16(uint8_t *out, uint16_t value, bool big_endian = false);
	static void put32(uint8_t *out, uint32_t value);
	static uint16_t signedValue(double value);
	static uint16_t unsignedValue(double value);
	static uint32_t bcd(uint32_t value);
	static void initPayload(uint8_t *out, uint8_t identifier);
	void encodeBattery(uint64_t now);
	void encodeGps(uint64_t now);
	void encodeVario(uint64_t now);
	void encodeAttitude(uint64_t now);
	void encodeText(uint64_t now);
	uint16_t climbRate(uint64_t window) const;

	Battery _battery{};
	Gps _gps{};
	Position _position{};
	Attitude _attitude{};
	Status _status{};
	HistorySample _history[HistorySize] {};
	size_t _history_head{0};
	size_t _history_size{0};
	Message _messages[QueueSize] {};
	uint32_t _dropped_messages{0};
	uint8_t _payloads[SlotCount][PayloadSize] {};
	uint8_t _gps_stats[PayloadSize] {};
	uint8_t _pending_gps_stats[PayloadSize] {};
	uint8_t _text[TextRows][PayloadSize] {};
	bool _active[SlotCount] {};
	bool _sent[SlotCount] {};
	uint64_t _last_sent[SlotCount] {};
	uint64_t _gps_pair_expires{0};
	uint8_t _next_slot{0};
	uint8_t _next_text_row{0};
	bool _gps_pending{false};
};

} // namespace srxl2
