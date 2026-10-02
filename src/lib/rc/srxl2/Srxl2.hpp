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

#include <math.h>
#include <stddef.h>
#include <stdint.h>

// Native single-receiver endpoint, based on Spektrum SRXL2 Specification Rev K.
// https://github.com/SpektrumRC/SRXL2/tree/6251dd14fbc95f262ecbc22e38ad39a8b907e411
namespace srxl2
{

static constexpr uint8_t DeviceId = 0x31;
// Request ten priority units for each of the eight stock telemetry records.
static constexpr uint8_t TelemetryPriority = 80;
static constexpr size_t MaxPacketSize = 80;
static constexpr size_t ChannelCount = 18;
static constexpr size_t TelemetrySize = 16;
static constexpr uint64_t BusTimeoutUs = 50000;
// Maximum age of a normal packet containing supported channel data. Individual
// omitted channels hold their last values (SRXL2 Rev K, section 7.7.1).
static constexpr uint64_t ChannelTimeoutUs = 100000;

struct Packet {
	uint8_t data[MaxPacketSize] {};
	uint8_t length {0};
	uint64_t first_sequence {0};
	uint64_t last_sequence {0};
};

uint16_t crc16(const uint8_t *data, size_t length);
uint16_t channel_value(uint16_t raw);
uint8_t channel_index(uint8_t wire_index);

// Timing bounds, not read()/work-queue timestamps: idle observation is an upper
// bound on final-byte arrival, and serialization takes at least the wire time.
uint64_t packet_start_upper_bound(uint64_t idle_us, size_t packet_length);
bool control_interval_too_short(uint64_t previous_start_lower_us, uint64_t current_idle_us,
				size_t current_packet_length);
bool control_interval_qualified(uint64_t previous_lower_us, uint64_t previous_upper_us,
				uint64_t current_lower_us, uint64_t current_upper_us);

// Qualify only adjacent, isolated control packets. Sequence gaps must never
// turn a skipped 5.5 ms packet into apparently supported 11 ms timing.
class ControlTiming
{
public:
	// Packet and burst sequence endpoints are half-open.
	void observe(uint64_t first, uint64_t end, uint64_t burst_first, uint64_t burst_end,
		     uint64_t start_lower_us, uint64_t idle_us, size_t packet_length);
	// Forget the timing baseline, while retaining a proven unsupported period.
	// Only two new qualified intervals can clear that verdict.
	void invalidate();
	bool qualified() const { return !_too_fast && _qualified_intervals >= 2; }
	bool too_fast() const { return _too_fast; }

private:
	uint64_t _previous_end {0};
	uint64_t _previous_lower_us {0};
	uint64_t _previous_upper_us {0};
	uint8_t _qualified_intervals {0};
	bool _have_previous {false};
	bool _too_fast {false};
};

class Parser
{
public:
	struct Statistics {
		uint32_t packets {0};
		uint32_t crc_errors {0};
		uint32_t length_errors {0};
		uint32_t discarded_bytes {0};
		uint64_t sequence_errors {0};
	};

	// Sequence identifies each received byte; packet endpoints are inclusive.
	bool feed(uint8_t byte, uint64_t sequence, Packet &packet);
	// Drain after feed() succeeds: resynchronization can expose another packet.
	bool next(Packet &packet);
	void reset();
	const Statistics &statistics() const { return _statistics; }

private:
	void discard(size_t count);
	uint8_t _buffer[MaxPacketSize] {};
	size_t _size {0};
	uint64_t _first_sequence {0};
	uint64_t _last_sequence {0};
	bool _have_sequence {false};
	Statistics _statistics {};
};

struct ChannelState {
	uint16_t values[ChannelCount] {};
	uint8_t channel_count {0};
	uint32_t valid_mask {0}; ///< Initialized normal values, including held channels.
	uint32_t fresh_mask {0}; ///< Updated within ChannelTimeoutUs; diagnostic only.
	uint64_t timestamp_last_signal {0};
	bool rc_lost {true};
	bool rc_failsafe {false};
	int32_t rssi {-1};
	float rssi_dbm {NAN};
	uint16_t lost_frame_count {0};
	uint16_t hold_count {0};
	uint16_t total_frame_count {0};
};

struct ReplyRequest {
	enum class Kind : uint8_t { None, Handshake, Telemetry };
	Kind kind {Kind::None};
	uint8_t data[22] {};
	uint8_t length {0};
};

struct Event {
	bool accepted {false};
	bool channel_data {false};
	bool channel_state_changed {false};
	ReplyRequest reply {};
};

class Endpoint
{
public:
	enum class State : uint8_t { Listening, Handshaking, Running };
	explicit Endpoint(uint32_t uid) : _uid(uid) {}
	Event process(const Packet &packet, uint64_t now_us);
	// Return true once when a previously active bus times out.
	bool tick(uint64_t now_us);
	ChannelState channel_state(uint64_t now_us) const;
	// Fill a telemetry request immediately before admission by the transport.
	// A null payload produces the standard telemetry-alive/no-data record.
	void make_telemetry(const uint8_t *payload, ReplyRequest &reply) const;
	void reset();
	State state() const { return _state; }
	uint8_t master_id() const { return _master_id; }
	uint64_t last_packet_time() const { return _last_packet_us; }

private:
	void clear_channels();
	void make_handshake(uint8_t destination, ReplyRequest &reply) const;
	void update_channels(const Packet &packet, uint64_t now_us);
	static bool fresh(uint64_t now_us, uint64_t then_us, uint64_t timeout_us);

	const uint32_t _uid;
	State _state {State::Listening};
	uint8_t _master_id {0};
	uint64_t _last_packet_us {0};
	bool _have_packet {false};
	uint16_t _values[ChannelCount] {};
	uint64_t _channel_updated[ChannelCount] {};
	uint32_t _seen_mask {0};
	uint32_t _normal_mask {0};
	uint64_t _last_signal_us {0};
	uint64_t _last_control_us {0};
	bool _failsafe {false};
	int32_t _rssi {-1};
	int8_t _rssi_dbm {0};
	uint64_t _rssi_time_us {0};
	uint64_t _rssi_dbm_time_us {0};
	bool _have_rssi {false};
	bool _have_rssi_dbm {false};
	uint16_t _lost_frame_count {0};
	uint16_t _hold_count {0};
	uint16_t _total_frame_count {0};
};

} // namespace srxl2
