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

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>

using namespace srxl2;

static Packet complete(std::vector<uint8_t> bytes, uint64_t sequence = 0)
{
	assert(bytes.size() + 2 <= MaxPacketSize);
	bytes[2] = bytes.size() + 2;
	const uint16_t crc = crc16(bytes.data(), bytes.size());
	bytes.push_back(crc >> 8);
	bytes.push_back(crc);
	Packet packet {};
	packet.length = bytes.size();
	memcpy(packet.data, bytes.data(), bytes.size());
	packet.first_sequence = sequence;
	packet.last_sequence = sequence + bytes.size() - 1;
	return packet;
}

static Packet handshake(uint8_t destination = DeviceId, uint8_t baud = 0, uint8_t source = 0x10)
{
	return complete({0xa6, 0x21, 0, source, destination, 10, baud, 3, 0x78, 0x56, 0x34, 0x12});
}

static Packet channels(uint32_t mask, uint8_t command = 0, uint8_t reply = DeviceId,
		       int8_t rssi = 88, uint16_t losses = 0x1234)
{
	std::vector<uint8_t> bytes {0xa6, 0xcd, 0, command, reply, static_cast<uint8_t>(rssi),
				    static_cast<uint8_t>(losses), static_cast<uint8_t>(losses >> 8),
				    static_cast<uint8_t>(mask), static_cast<uint8_t>(mask >> 8),
				    static_cast<uint8_t>(mask >> 16), static_cast<uint8_t>(mask >> 24)};

	for (unsigned i = 0; i < 32; ++i) {
		if (mask & (uint32_t(1) << i)) {
			const uint16_t raw = 0x8000 + i * 32;
			bytes.push_back(raw);
			bytes.push_back(raw >> 8);
		}
	}

	return complete(bytes);
}

static void append(Parser &parser, const Packet &source, uint64_t &sequence, std::vector<Packet> &packets)
{
	for (size_t i = 0; i < source.length; ++i) {
		Packet result {};

		if (parser.feed(source.data[i], sequence++, result)) {
			packets.push_back(result);

			while (parser.next(result)) {
				packets.push_back(result);
			}
		}
	}
}

static void test_crc_and_scaling()
{
	const uint8_t check[] = "123456789";
	assert(crc16(check, sizeof(check) - 1) == 0x31c3);
	assert(crc16(check, 0) == 0);
	assert(channel_value(0) == 903);
	assert(channel_value(0x8000) == 1500);
	assert(channel_value(0xfffc) == 2096);
	assert(channel_value(0x2aa0) == 1102);
	assert(channel_value(0xd554) == 1898);

	for (unsigned i = 0; i < 2048; ++i) {
		assert(channel_value(i << 5) == 903 + lroundf(i * 0.583f));
	}

	assert(channel_index(0) == 2 && channel_index(1) == 0 && channel_index(2) == 1);
	assert(channel_index(3) == 3 && channel_index(31) == 31);
}

static void test_control_timing_bounds()
{
	constexpr uint64_t previous_start = 100000;
	// 11 ms start-to-start: shorten the next frame from seven channels to a
	// zero-mask fade. The old end-to-end measurement would be only 9785 us.
	constexpr uint64_t previous_idle = previous_start + 2431 + 87;
	constexpr uint64_t current_idle = previous_start + 11000 + 1216 + 87;
	assert(current_idle - previous_idle < 10000);
	assert(!control_interval_too_short(previous_start, current_idle, 14));
	assert(control_interval_qualified(previous_start, previous_start + 88,
					  previous_start + 11000, packet_start_upper_bound(current_idle, 14)));
	assert(control_interval_qualified(previous_start, previous_start + 88,
					  previous_start + 22000, previous_start + 22088));

	const uint64_t fast_idle = previous_start + 5500 + 1216 + 87;
	assert(control_interval_too_short(previous_start, fast_idle, 14));
	assert(!control_interval_qualified(previous_start, previous_start + 88,
					   previous_start + 5500, packet_start_upper_bound(fast_idle, 14)));

	// Late work-queue processing does not enter these calculations. A delayed
	// idle interrupt or uncertain previous RX start can only withhold a verdict.
	assert(!control_interval_too_short(previous_start, fast_idle + 10000, 14));
	assert(!control_interval_too_short(previous_start - 10000, fast_idle, 14));
	assert(!control_interval_qualified(previous_start, previous_start + 5000,
					   previous_start + 11000, previous_start + 11088));
	assert(!control_interval_qualified(previous_start - 30000, previous_start + 88,
					   previous_start + 11000, previous_start + 11088));

	assert(packet_start_upper_bound(100, 14) == 0);
	assert(packet_start_upper_bound(10000, 0) == 0);
	assert(packet_start_upper_bound(10000, 81) == 0);
	assert(!control_interval_too_short(0, fast_idle, 14));
	assert(!control_interval_qualified(previous_start, previous_start - 1,
					   previous_start + 11000, previous_start + 11088));
	assert(!control_interval_qualified(previous_start, previous_start + 88,
					   previous_start + 11000, previous_start + 10999));
}

static void observe_control(ControlTiming &timing, uint64_t sequence, uint64_t start, size_t length)
{
	// Known bounds: first-byte IRQ uncertainty of 100 us and one character
	// between final-byte arrival and the observed idle interrupt.
	const uint64_t idle = start + (length * 10000000 + 115199) / 115200 + 87;
	timing.observe(sequence, sequence + length, sequence, sequence + length, start - 100, idle, length);
}

static void test_control_timing_qualification()
{
	ControlTiming timing;
	observe_control(timing, 0, 100000, 28);
	assert(!timing.qualified());
	observe_control(timing, 28, 111000, 14);
	assert(!timing.qualified());
	observe_control(timing, 42, 122000, 28);
	assert(timing.qualified() && !timing.too_fast());

	// Dropping alternate 5.5 ms packets leaves an apparent 11 ms interval,
	// but byte sequence gaps prevent both observed intervals from qualifying.
	timing = {};
	observe_control(timing, 0, 100000, 28);
	observe_control(timing, 56, 111000, 28);
	observe_control(timing, 112, 122000, 28);
	assert(!timing.qualified() && !timing.too_fast());
	observe_control(timing, 140, 127500, 28);
	assert(!timing.qualified() && timing.too_fast());
	timing.invalidate();
	observe_control(timing, 168, 138500, 28);
	observe_control(timing, 196, 149500, 28);
	assert(!timing.qualified() && timing.too_fast());
	observe_control(timing, 224, 160500, 28);
	assert(timing.qualified() && !timing.too_fast());

	// A missing IDLE timestamp, partial/coalesced burst, or inconsistent byte
	// range resets qualification. No old observation bridges that uncertainty.
	timing = {};
	observe_control(timing, 0, 100000, 28);
	observe_control(timing, 28, 111000, 28);
	timing.observe(56, 84, 56, 84, 121900, 0, 28);
	observe_control(timing, 84, 133000, 28);
	assert(!timing.qualified());
	observe_control(timing, 112, 144000, 28);
	assert(!timing.qualified());
	observe_control(timing, 140, 155000, 28);
	assert(timing.qualified());
	timing.observe(168, 196, 167, 196, 165900, 168518, 28);
	assert(!timing.qualified());
	observe_control(timing, 196, 177000, 28);
	timing.observe(224, 252, 224, 253, 187900, 190518, 28);
	assert(!timing.qualified());
	observe_control(timing, 252, 199000, 28);
	timing.observe(280, 308, 280, 308, 209900, 212518, 14);
	assert(!timing.qualified());
	timing.invalidate();
	observe_control(timing, 308, 221000, 28);
	observe_control(timing, 336, 243000, 28);
	assert(!timing.qualified());
	observe_control(timing, 364, 265000, 28);
	assert(timing.qualified());
}

static void test_control_timing_recovery()
{
	for (const uint64_t period : {11000, 22000}) {
		ControlTiming timing;
		uint64_t sequence = 0;
		uint64_t start = 100000;
		auto observe = [&](size_t length) {
			observe_control(timing, sequence, start, length);
			sequence += length;
		};

		// Short zero-channel packets before RF acquisition must not latch
		// telemetry off after normal 11/22 ms channel traffic begins.
		for (unsigned i = 0; i < 10; ++i) {
			observe(14);
			assert(!timing.qualified());
			start += 5500;
		}

		assert(timing.too_fast());
		observe(28); // First RF packet can still have a short preceding interval.
		assert(!timing.qualified());
		start += period;
		observe(28);
		assert(!timing.qualified() && timing.too_fast());
		start += period;
		observe(28);
		assert(timing.qualified() && !timing.too_fast());

		// Another short interval immediately suppresses replies again.
		start += 5500;
		observe(14);
		assert(!timing.qualified() && timing.too_fast());

		// Missing alternate 5.5 ms packets must not look like recovery.
		for (unsigned i = 0; i < 10; ++i) {
			sequence += 14;
			start += 11000;
			observe(14);
			assert(!timing.qualified() && timing.too_fast());
		}

		// Silence/invalid timing alone cannot clear the fast-stream verdict.
		timing.invalidate();
		start += BusTimeoutUs;
		observe(28);
		start += period;
		observe(28);
		assert(!timing.qualified() && timing.too_fast());
		timing.invalidate();
		start += period;
		observe(28);
		start += period;
		observe(28);
		assert(!timing.qualified() && timing.too_fast());
		start += period;
		observe(28);
		assert(timing.qualified() && !timing.too_fast());
	}
}

static void test_stream_parser()
{
	Parser parser;
	Packet result {};
	uint64_t sequence = uint64_t(UINT32_MAX) + 123;
	const uint8_t noise[] {0, 0xa6, 0xcd, 0xff};

	for (uint8_t byte : noise) {
		assert(!parser.feed(byte, sequence++, result));
	}

	const uint64_t first = sequence;
	std::vector<Packet> packets;
	append(parser, channels(0x7f), sequence, packets);
	append(parser, handshake(), sequence, packets);
	assert(packets.size() == 2);
	assert(packets[0].first_sequence == first);
	assert(packets[0].last_sequence == first + 27);
	assert(packets[1].first_sequence == first + 28);
	assert(parser.statistics().length_errors == 1);

	Packet corrupt = channels(0xf);
	corrupt.data[corrupt.length - 1] ^= 1;
	append(parser, corrupt, sequence, packets);
	append(parser, channels(0xffffffff), sequence, packets);
	assert(packets.size() == 3 && packets.back().length == 78);
	assert(parser.statistics().crc_errors >= 1);

	// A missing byte must invalidate a partial frame instead of stitching data.
	const Packet partial = channels(0x7f);
	assert(!parser.feed(partial.data[0], sequence++, result));
	assert(!parser.feed(partial.data[1], sequence++, result));
	sequence += 10;
	append(parser, partial, sequence, packets);
	assert(packets.size() == 4);
	assert(parser.statistics().sequence_errors == 1);

	// Full-sized irrelevant frames are safely framed, without being RC data.
	std::vector<uint8_t> maximum(78, 0x55);
	maximum[0] = 0xa6;
	maximum[1] = 0x99;
	const Packet maximum_packet = complete(maximum);
	append(parser, maximum_packet, sequence, packets);
	assert(packets.back().length == MaxPacketSize);
	Endpoint endpoint(123);
	assert(!endpoint.process(packets.back(), 10).accepted);
}

static void test_resynchronization()
{
	Parser parser;
	Packet result {};
	const Packet inner = handshake();
	std::vector<uint8_t> outer {0xa6, 0xcd, 40};
	outer.insert(outer.end(), inner.data, inner.data + inner.length);
	outer.insert(outer.end(), inner.data, inner.data + inner.length);
	outer.resize(40, 0);
	uint64_t sequence = 123;
	std::vector<Packet> packets;

	for (uint8_t byte : outer) {
		if (parser.feed(byte, sequence++, result)) {
			packets.push_back(result);

			while (parser.next(result)) {
				packets.push_back(result);
			}
		}
	}

	assert(packets.size() == 2);
	assert(packets[0].first_sequence == 126);
	assert(packets[1].first_sequence == 140);
	assert(parser.statistics().crc_errors == 1);
}

static void test_handshake_and_restart()
{
	Endpoint endpoint(0x12345678);
	Event event = endpoint.process(handshake(0x40), 1);
	assert(event.accepted && event.reply.kind == ReplyRequest::Kind::None);
	assert(endpoint.master_id() == 0);
	assert(endpoint.channel_state(1).rc_lost);
	event = endpoint.process(handshake(), 2);
	assert(event.accepted && !event.channel_data);
	assert(event.reply.kind == ReplyRequest::Kind::Handshake);
	assert(event.reply.length == 14);
	const uint8_t expected_prefix[] {0xa6, 0x21, 14, 0x31, 0x10, 80, 0, 0, 0x78, 0x56, 0x34, 0x12};
	assert(memcmp(event.reply.data, expected_prefix, sizeof(expected_prefix)) == 0);
	assert(crc16(event.reply.data, event.reply.length) == 0);
	assert(endpoint.state() == Endpoint::State::Handshaking);
	assert(!endpoint.process(handshake(0xff, 1), 3).accepted);
	assert(endpoint.process(handshake(0xff), 4).accepted);
	assert(endpoint.state() == Endpoint::State::Running);
	assert(!endpoint.process(handshake(0x10, 0, DeviceId), 5).accepted); // our own echo

	ReplyRequest reply;
	uint8_t payload[TelemetrySize] {};
	payload[0] = 0x7f;
	endpoint.make_telemetry(payload, reply);
	assert(reply.length == 22 && reply.data[3] == 0x10 && reply.data[4] == 0x7f);
	assert(crc16(reply.data, reply.length) == 0);
	assert(!endpoint.tick(50003));
	assert(endpoint.tick(50004));
	assert(!endpoint.tick(50005));
	assert(endpoint.master_id() == 0 && endpoint.state() == Endpoint::State::Listening);

	// FC-only restart on an already-running bus must not require a handshake.
	endpoint.reset();
	event = endpoint.process(channels(0x7f), 100);
	assert(event.channel_data && event.reply.kind == ReplyRequest::Kind::Telemetry);
	assert(!endpoint.channel_state(100).rc_lost);
	endpoint.make_telemetry(nullptr, reply);
	assert(reply.data[3] == 0xff && reply.data[4] == 0);
	assert(endpoint.process(handshake(0xff), 200).accepted);
	endpoint.make_telemetry(nullptr, reply);
	assert(reply.data[3] == 0x10);
	assert(endpoint.tick(50200));
	assert(endpoint.channel_state(50200).timestamp_last_signal == 100);
	assert(endpoint.channel_state(50200).channel_count == 0);
}

static void test_channels_and_freshness()
{
	Endpoint endpoint(123);
	endpoint.process(channels(0x7f), 1);
	ChannelState state = endpoint.channel_state(1);
	assert(!state.rc_lost && !state.rc_failsafe);
	assert(state.channel_count == 7 && state.valid_mask == 0x7f);
	assert(state.values[2] == 1500);
	assert(state.values[0] == channel_value(0x8020));
	assert(state.values[1] == channel_value(0x8040));
	assert(state.lost_frame_count == 0x1234 && state.rssi == 88);
	assert(isnan(state.rssi_dbm));

	// Normal partial updates hold omitted auxiliary values (spec section 7.7.1).
	for (uint64_t now = 20001; now <= 100001; now += 20000) {
		endpoint.process(channels(0xf, 0, 0, -63), now);
	}

	state = endpoint.channel_state(100001);
	assert(!state.rc_lost && state.timestamp_last_signal == 100001);
	assert(state.valid_mask == 0x7f && state.fresh_mask == 0xf);
	assert(state.values[6] == channel_value(0x8000 + 6 * 32));
	assert(state.rssi == -1 && fabsf(state.rssi_dbm + 63.f) < .001f);
	endpoint.process(channels(0x70), 110000);
	state = endpoint.channel_state(110000);
	assert(!state.rc_lost && state.timestamp_last_signal == 110000);

	// Thirty-two-bit masks consume unsupported values safely and cap at18.
	endpoint.reset();
	endpoint.process(channels(0xffffffff), 1);
	state = endpoint.channel_state(1);
	assert(!state.rc_lost && state.channel_count == ChannelCount);
	assert(state.values[17] == channel_value(0x8000 + 17 * 32));
	endpoint.process(channels(uint32_t(1) << 31), 2);
	assert(endpoint.channel_state(2).timestamp_last_signal == 1);

	// Initial partial data must not advertise uninitialized channels as usable.
	endpoint.reset();
	endpoint.process(channels(0x9), 1);
	assert(endpoint.channel_state(1).rc_lost);
	endpoint.process(channels(0x6), 2);
	assert(!endpoint.channel_state(2).rc_lost);
}

static void test_held_channels_and_non_control_traffic()
{
	Endpoint endpoint(123);
	endpoint.process(channels(0x3fff), 1);

	// Reproduce a receiver that supplies 14 channels initially, then sends
	// only a subset for longer than the old per-channel timeout.
	for (uint64_t now = 22001; now <= 2200001; now += 22000) {
		endpoint.process(channels(0x3f), now);
		const auto state = endpoint.channel_state(now);
		assert(!state.rc_lost && state.channel_count == 14);
		assert(state.timestamp_last_signal == now);
		assert(state.valid_mask == 0x3fff);
		assert(state.values[13] == channel_value(0x8000 + 13 * 32));

		if (now >= 100001) {
			assert(state.fresh_mask == 0x3f);
		}
	}

	// Discovery traffic keeps the bus active but must not preserve RF health.
	const uint64_t last_signal = endpoint.channel_state(2200001).timestamp_last_signal;

	for (uint64_t elapsed = 20000; elapsed <= ChannelTimeoutUs; elapsed += 20000) {
		endpoint.process(handshake(0xff), last_signal + elapsed);
	}

	assert(endpoint.channel_state(last_signal + ChannelTimeoutUs).rc_lost);
	assert(endpoint.channel_state(last_signal + ChannelTimeoutUs).timestamp_last_signal == last_signal);

	// Channel updates entirely above our 18-channel limit cannot refresh RC.
	endpoint.reset();
	endpoint.process(channels(0xf), 1);

	for (uint64_t now = 20001; now <= 100001; now += 20000) {
		endpoint.process(channels(uint32_t(1) << 31), now);
	}

	assert(endpoint.channel_state(100001).rc_lost);
	assert(endpoint.channel_state(100001).timestamp_last_signal == 1);
}

static void test_fade_and_failsafe()
{
	Endpoint endpoint(123);
	endpoint.process(channels(0xf), 1);

	for (uint64_t now = 20001; now <= 100001; now += 20000) {
		endpoint.process(channels(0), now);
	}

	assert(endpoint.channel_state(100001).rc_lost);
	assert(endpoint.channel_state(100001).timestamp_last_signal == 1);

	endpoint.reset();
	endpoint.process(channels(0x7f), 1);
	Event event = endpoint.process(channels(1, 1, 0, 0, 0x7777), 2);
	assert(event.channel_data);
	ChannelState state = endpoint.channel_state(2);
	assert(state.rc_failsafe && state.rc_lost);
	assert(state.values[2] == 1500 && state.values[0] == 0 && state.values[6] == 0);
	assert(state.lost_frame_count == 0x1234 && state.hold_count == 0x7777 && state.timestamp_last_signal == 1);
	endpoint.process(channels(0), 3);
	assert(endpoint.channel_state(3).rc_failsafe);
	endpoint.process(channels(0xf), 4);
	assert(endpoint.channel_state(4).rc_lost);
	endpoint.process(channels(0x70), 5);
	state = endpoint.channel_state(5);
	assert(!state.rc_lost && !state.rc_failsafe && state.timestamp_last_signal == 5);
}

static void test_receiver_counters()
{
	Endpoint endpoint(123);
	endpoint.process(channels(0xf, 0, 0, 80, 0xffff), 1);
	endpoint.process(channels(0, 1, 0, 0, 0xffff), 2);
	ChannelState state = endpoint.channel_state(2);
	assert(state.lost_frame_count == 0xffff && state.hold_count == 0xffff);

	// These are independent receiver-provided 16-bit counters. Accept wrap
	// and receiver restart as values, without synthesizing a loss delta.
	endpoint.process(channels(0xf, 0, 0, 80, 0), 3);
	state = endpoint.channel_state(3);
	assert(state.lost_frame_count == 0 && state.hold_count == 0xffff);
	endpoint.process(channels(0, 1, 0, 0, 0), 4);
	state = endpoint.channel_state(4);
	assert(state.lost_frame_count == 0 && state.hold_count == 0);
	assert(state.total_frame_count == 4);

	endpoint.process(channels(0xf, 0, 0, 80, 123), 5);
	endpoint.process(channels(0, 1, 0, 0, 456), 6);
	endpoint.reset();
	state = endpoint.channel_state(7);
	assert(state.lost_frame_count == 0 && state.hold_count == 0 && state.total_frame_count == 0);
}

static void test_invalid_packets_do_not_change_state()
{
	Endpoint endpoint(123);
	endpoint.process(channels(0x7f), 1);
	Packet packet = channels(0x7f);
	packet.data[packet.length - 1] ^= 1;
	assert(!endpoint.process(packet, 2).accepted);
	packet = complete({0xa6, 0xcd, 0, 0, DeviceId}); // valid CRC, missing payload
	assert(!endpoint.process(packet, 3).accepted);
	packet = complete({0xa6, 0xcd, 0, 0, DeviceId, 90, 0, 0, 1, 0, 0, 0}); // missing mask value
	assert(!endpoint.process(packet, 4).accepted);
	packet = channels(0x7f, 2); // VTX must never become channel data
	assert(!endpoint.process(packet, 5).accepted);
	packet = handshake();
	packet.length = 255;
	assert(!endpoint.process(packet, 6).accepted);
	const ChannelState state = endpoint.channel_state(6);
	assert(state.timestamp_last_signal == 1 && state.total_frame_count == 1);
	assert(state.rssi == 88 && !state.rc_lost);
}

static void test_deterministic_noise()
{
	Parser parser;
	Endpoint endpoint(123);
	Packet packet {};
	uint32_t random = 0x184657ab;
	uint64_t sequence = 0;

	for (unsigned i = 0; i < 100000; ++i) {
		random = random * 1664525 + 1013904223;

		if (parser.feed(random >> 24, sequence++, packet)) {
			assert(packet.length >= 5 && packet.length <= MaxPacketSize);
			assert(crc16(packet.data, packet.length) == 0);
			endpoint.process(packet, i);
		}
	}

	parser.reset();
	std::vector<Packet> packets;
	append(parser, channels(0x7f), sequence, packets);
	assert(packets.size() == 1);
	assert(endpoint.process(packets[0], 100001).accepted);
}

int main()
{
	test_crc_and_scaling();
	test_control_timing_bounds();
	test_control_timing_qualification();
	test_control_timing_recovery();
	test_stream_parser();
	test_resynchronization();
	test_handshake_and_restart();
	test_channels_and_freshness();
	test_held_channels_and_non_control_traffic();
	test_fade_and_failsafe();
	test_receiver_counters();
	test_invalid_packets_do_not_change_state();
	test_deterministic_noise();
	puts("SRXL2 protocol tests passed");
	return 0;
}
