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

#include "Srxl2.hpp"

#include <string.h>

namespace srxl2
{
namespace
{
constexpr uint8_t Header = 0xa6;
constexpr uint8_t Handshake = 0x21;
constexpr uint8_t Telemetry = 0x80;
constexpr uint8_t Control = 0xcd;
constexpr uint8_t NormalChannels = 0;
constexpr uint8_t FailsafeChannels = 1;
constexpr uint8_t Broadcast = 0xff;
constexpr size_t HeaderSize = 3;
constexpr size_t CrcSize = 2;
constexpr size_t HandshakeSize = 14;
constexpr size_t ChannelBaseSize = 14;
constexpr size_t MaskOffset = 8;
constexpr size_t ValuesOffset = 12;
constexpr uint8_t MinimumChannels = 4;

uint16_t get_le16(const uint8_t *bytes)
{
	return uint16_t(bytes[0]) | (uint16_t(bytes[1]) << 8);
}

uint32_t get_le32(const uint8_t *bytes)
{
	return uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) | (uint32_t(bytes[2]) << 16)
	       | (uint32_t(bytes[3]) << 24);
}

void put_le32(uint8_t *bytes, uint32_t value)
{
	for (size_t i = 0; i < sizeof(value); ++i) {
		bytes[i] = value >> (8 * i);
	}
}

unsigned bit_count(uint32_t mask)
{
	unsigned count = 0;

	while (mask != 0) {
		mask &= mask - 1;
		++count;
	}

	return count;
}

bool valid_crc(const uint8_t *bytes, size_t length)
{
	const uint16_t received = (uint16_t(bytes[length - CrcSize]) << 8) | bytes[length - 1];
	return received == crc16(bytes, length - CrcSize);
}

void finish_reply(ReplyRequest &reply)
{
	const uint16_t crc = crc16(reply.data, reply.length - CrcSize);
	reply.data[reply.length - CrcSize] = crc >> 8;
	reply.data[reply.length - 1] = crc;
}

bool receiver_id(uint8_t id)
{
	return id >= 0x10 && id <= 0x2f && id != 0x20;
}
} // namespace

uint16_t crc16(const uint8_t *data, size_t length)
{
	uint16_t crc = 0;

	for (size_t i = 0; i < length; ++i) {
		crc ^= uint16_t(data[i]) << 8;

		for (unsigned bit = 0; bit < 8; ++bit) {
			crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
		}
	}

	return crc;
}

uint16_t channel_value(uint16_t raw)
{
	// SRXL2 Rev K recommends shifting to DSM's 11-bit resolution. Match PX4
	// DSM's round(servo_position * 0.583) + 903 microsecond conversion.
	return 903 + ((uint32_t(raw >> 5) * 583 + 500) / 1000);
}

uint8_t channel_index(uint8_t wire_index)
{
	switch (wire_index) {
	case 0: return 2; // throttle

	case 1: return 0; // roll

	case 2: return 1; // pitch

	default: return wire_index;
	}
}

uint64_t packet_start_upper_bound(uint64_t idle_us, size_t packet_length)
{
	if (packet_length < HeaderSize + CrcSize || packet_length > MaxPacketSize) {
		return 0;
	}

	// Round down the duration being subtracted to preserve an upper bound.
	constexpr uint64_t Baudrate = 115200;
	constexpr uint64_t BitMicrosecondsPerByte = 10 * 1000000;
	const uint64_t wire_time_us = packet_length * BitMicrosecondsPerByte / Baudrate;
	return idle_us > wire_time_us ? idle_us - wire_time_us : 0;
}

bool control_interval_too_short(uint64_t previous_start_lower_us, uint64_t current_idle_us,
				size_t current_packet_length)
{
	constexpr uint64_t FastIntervalThresholdUs = 10000;
	const uint64_t upper = packet_start_upper_bound(current_idle_us, current_packet_length);
	return previous_start_lower_us != 0 && upper > previous_start_lower_us
	       && upper - previous_start_lower_us < FastIntervalThresholdUs;
}

bool control_interval_qualified(uint64_t previous_lower_us, uint64_t previous_upper_us,
				uint64_t current_lower_us, uint64_t current_upper_us)
{
	constexpr uint64_t MinimumIntervalLowerUs = 9000;
	constexpr uint64_t FastIntervalThresholdUs = 10000;
	constexpr uint64_t MaximumIntervalUpperUs = 30000;

	if (previous_lower_us == 0 || previous_upper_us < previous_lower_us
	    || current_upper_us < current_lower_us || current_lower_us <= previous_upper_us) {
		return false;
	}

	const uint64_t interval_lower_us = current_lower_us - previous_upper_us;
	const uint64_t interval_upper_us = current_upper_us - previous_lower_us;
	return interval_lower_us >= MinimumIntervalLowerUs && interval_upper_us >= FastIntervalThresholdUs
	       && interval_upper_us <= MaximumIntervalUpperUs;
}

void ControlTiming::invalidate()
{
	_have_previous = false;
	_qualified_intervals = 0;
}

void ControlTiming::observe(uint64_t first, uint64_t end, uint64_t burst_first, uint64_t burst_end,
			    uint64_t start_lower_us, uint64_t idle_us, size_t packet_length)
{
	const uint64_t start_upper_us = packet_start_upper_bound(idle_us, packet_length);

	if (first != burst_first || end != burst_end || end <= first || end - first != packet_length
	    || start_lower_us == 0 || start_upper_us < start_lower_us) {
		invalidate();
		return;
	}

	if (_have_previous && first == _previous_end) {
		if (control_interval_too_short(_previous_lower_us, idle_us, packet_length)) {
			_too_fast = true;
		}

		if (control_interval_qualified(_previous_lower_us, _previous_upper_us,
					       start_lower_us, start_upper_us)) {
			if (_qualified_intervals < 2) {
				++_qualified_intervals;
			}

			// Startup or RF-loss traffic can have a different cadence from
			// normal channel data. Require the same complete qualification
			// as startup before clearing an earlier fast-stream verdict.
			if (_qualified_intervals >= 2) {
				_too_fast = false;
			}

		} else {
			_qualified_intervals = 0;
		}

	} else {
		_qualified_intervals = 0;
	}

	_previous_end = end;
	_previous_lower_us = start_lower_us;
	_previous_upper_us = start_upper_us;
	_have_previous = true;
}

void Parser::reset()
{
	_size = 0;
	_have_sequence = false;
}

void Parser::discard(size_t count)
{
	memmove(_buffer, _buffer + count, _size - count);
	_size -= count;
	_first_sequence += count;
}

bool Parser::feed(uint8_t byte, uint64_t sequence, Packet &packet)
{
	if (_have_sequence && sequence != _last_sequence + 1) {
		++_statistics.sequence_errors;
		_statistics.discarded_bytes += _size;
		reset();
	}

	_have_sequence = true;
	_last_sequence = sequence;

	if (_size == MaxPacketSize) {
		discard(1);
		++_statistics.discarded_bytes;
	}

	if (_size == 0) {
		_first_sequence = sequence;
	}

	_buffer[_size++] = byte;
	return next(packet);
}

bool Parser::next(Packet &packet)
{
	while (_size > 0) {
		if (_buffer[0] != Header) {
			discard(1);
			++_statistics.discarded_bytes;
			continue;
		}

		if (_size < HeaderSize) {
			return false;
		}

		const uint8_t length = _buffer[2];

		if (length < HeaderSize + CrcSize || length > MaxPacketSize) {
			discard(1);
			++_statistics.length_errors;
			++_statistics.discarded_bytes;
			continue;
		}

		if (_size < length) {
			return false;
		}

		if (!valid_crc(_buffer, length)) {
			discard(1);
			++_statistics.crc_errors;
			++_statistics.discarded_bytes;
			continue;
		}

		packet = {};
		packet.length = length;
		packet.first_sequence = _first_sequence;
		packet.last_sequence = _first_sequence + length - 1;
		memcpy(packet.data, _buffer, length);
		discard(length);
		++_statistics.packets;
		return true;
	}

	return false;
}

bool Endpoint::fresh(uint64_t now_us, uint64_t then_us, uint64_t timeout_us)
{
	return now_us >= then_us && now_us - then_us < timeout_us;
}

void Endpoint::clear_channels()
{
	memset(_values, 0, sizeof(_values));
	memset(_channel_updated, 0, sizeof(_channel_updated));
	_seen_mask = 0;
	_normal_mask = 0;
	_last_control_us = 0;
	_have_rssi = false;
	_have_rssi_dbm = false;
}

void Endpoint::reset()
{
	_state = State::Listening;
	_master_id = 0;
	_last_packet_us = 0;
	_have_packet = false;
	_last_signal_us = 0;
	_failsafe = false;
	_lost_frame_count = 0;
	_hold_count = 0;
	_total_frame_count = 0;
	clear_channels();
}

bool Endpoint::tick(uint64_t now_us)
{
	if (_have_packet && !fresh(now_us, _last_packet_us, BusTimeoutUs)) {
		_have_packet = false;
		_master_id = 0;
		_state = State::Listening;
		clear_channels();
		return true;
	}

	return false;
}

void Endpoint::make_handshake(uint8_t destination, ReplyRequest &reply) const
{
	reply = {};
	reply.kind = ReplyRequest::Kind::Handshake;
	reply.length = HandshakeSize;
	reply.data[0] = Header;
	reply.data[1] = Handshake;
	reply.data[2] = reply.length;
	reply.data[3] = DeviceId;
	reply.data[4] = destination;
	reply.data[5] = TelemetryPriority;
	// Baud support and device info remain zero: 115200 only, no RF or FP.
	put_le32(&reply.data[8], _uid);
	finish_reply(reply);
}

void Endpoint::make_telemetry(const uint8_t *payload, ReplyRequest &reply) const
{
	reply = {};
	reply.kind = ReplyRequest::Kind::Telemetry;
	reply.length = sizeof(reply.data);
	reply.data[0] = Header;
	reply.data[1] = Telemetry;
	reply.data[2] = reply.length;
	// A broadcast destination requests a handshake after an FC-only reboot.
	// Destination zero would disable every receiver's RF telemetry.
	reply.data[3] = _master_id != 0 ? _master_id : Broadcast;

	if (payload != nullptr) {
		memcpy(&reply.data[4], payload, TelemetrySize);
	}

	finish_reply(reply);
}

Event Endpoint::process(const Packet &packet, uint64_t now_us)
{
	Event event {};

	if (packet.length < HeaderSize + CrcSize || packet.length > MaxPacketSize
	    || packet.data[0] != Header || packet.data[2] != packet.length
	    || !valid_crc(packet.data, packet.length)) {
		return event;
	}

	if (packet.data[1] == Handshake) {
		if (packet.length != HandshakeSize || !receiver_id(packet.data[3])) {
			return event;
		}

		const uint8_t destination = packet.data[4];

		if (destination == Broadcast && packet.data[6] != 0) {
			// Never silently run at the wrong baud after unsupported negotiation.
			return event;
		}

		event.channel_state_changed = tick(now_us);

		if (destination == DeviceId || destination == Broadcast) {
			_master_id = packet.data[3];
			_state = destination == Broadcast ? State::Running : State::Handshaking;

			if (destination == DeviceId) {
				make_handshake(_master_id, event.reply);
			}
		}

		event.accepted = true;

	} else if (packet.data[1] == Control) {
		if (packet.length < ChannelBaseSize || (packet.data[3] != NormalChannels
							&& packet.data[3] != FailsafeChannels)) {
			return event;
		}

		const uint32_t mask = get_le32(&packet.data[MaskOffset]);

		if (packet.length != ChannelBaseSize + sizeof(uint16_t) * bit_count(mask)) {
			return event;
		}

		event.channel_state_changed = tick(now_us);
		_state = State::Running;
		_last_packet_us = now_us;
		_have_packet = true;
		update_channels(packet, now_us);
		event.accepted = true;
		event.channel_data = true;
		event.channel_state_changed = true;

		if (packet.data[4] == DeviceId) {
			event.reply.kind = ReplyRequest::Kind::Telemetry;
		}

	} else {
		// Bind, VTX, forward programming and other bus devices are out of scope.
		return event;
	}

	_last_packet_us = now_us;
	_have_packet = true;
	return event;
}

void Endpoint::update_channels(const Packet &packet, uint64_t now_us)
{
	const bool failsafe = packet.data[3] == FailsafeChannels;
	const uint32_t mask = get_le32(&packet.data[MaskOffset]);
	const int8_t rssi = static_cast<int8_t>(packet.data[5]);
	++_total_frame_count;

	if (rssi < 0) {
		_rssi_dbm = rssi;
		_rssi_dbm_time_us = now_us;
		_have_rssi_dbm = true;

	} else if (rssi <= 100) {
		_rssi = rssi;
		_rssi_time_us = now_us;
		_have_rssi = true;

	} else {
		_have_rssi = false;
	}

	if (failsafe) {
		// The counter in a failsafe frame means holds, not frame losses.
		_hold_count = get_le16(&packet.data[6]);
		_failsafe = true;
		_normal_mask = 0;
		_last_control_us = 0;
		memset(_values, 0, sizeof(_values));
		memset(_channel_updated, 0, sizeof(_channel_updated));

	} else {
		_lost_frame_count = get_le16(&packet.data[6]);
	}

	size_t offset = ValuesOffset;
	bool supported_update = false;

	for (uint8_t wire_index = 0; wire_index < 32; ++wire_index) {
		if ((mask & (uint32_t(1) << wire_index)) == 0) {
			continue;
		}

		const uint16_t value = get_le16(&packet.data[offset]);
		offset += sizeof(uint16_t);
		const uint8_t index = channel_index(wire_index);

		if (index < ChannelCount) {
			const uint32_t bit = uint32_t(1) << index;
			_values[index] = channel_value(value);
			_seen_mask |= bit;

			if (!failsafe) {
				_channel_updated[index] = now_us;
				_normal_mask |= bit;
				supported_update = true;
			}
		}
	}

	// A zero mask denotes a fade or no RF acquisition. Neither that packet nor
	// a packet containing only unsupported channels can restore a valid signal.
	if (supported_update && !failsafe) {
		_failsafe = false;
		_last_control_us = now_us;

		if (!channel_state(now_us).rc_lost) {
			_last_signal_us = now_us;
		}
	}
}

ChannelState Endpoint::channel_state(uint64_t now_us) const
{
	ChannelState state {};
	memcpy(state.values, _values, sizeof(state.values));
	state.timestamp_last_signal = _last_signal_us;
	state.rc_failsafe = _failsafe;
	state.lost_frame_count = _lost_frame_count;
	state.hold_count = _hold_count;
	state.total_frame_count = _total_frame_count;

	for (size_t i = 0; i < ChannelCount; ++i) {
		const uint32_t bit = uint32_t(1) << i;

		if (_seen_mask & bit) {
			state.channel_count = i + 1;
		}

		if (_normal_mask & bit) {
			// Normal channel packets are partial updates. An omitted channel
			// remains usable until failsafe or bus reset, not just for 100 ms.
			state.valid_mask |= bit;

			if (fresh(now_us, _channel_updated[i], ChannelTimeoutUs)) {
				state.fresh_mask |= bit;
			}
		}
	}

	const uint32_t expected_mask = (uint32_t(1) << state.channel_count) - 1;
	state.rc_lost = _failsafe || !_have_packet || !fresh(now_us, _last_packet_us, BusTimeoutUs)
			|| !fresh(now_us, _last_control_us, ChannelTimeoutUs)
			|| state.channel_count < MinimumChannels || state.valid_mask != expected_mask;

	if (_have_rssi && fresh(now_us, _rssi_time_us, ChannelTimeoutUs)) {
		state.rssi = _rssi;
	}

	if (_have_rssi_dbm && fresh(now_us, _rssi_dbm_time_us, ChannelTimeoutUs)) {
		state.rssi_dbm = _rssi_dbm;
	}

	return state;
}

} // namespace srxl2
