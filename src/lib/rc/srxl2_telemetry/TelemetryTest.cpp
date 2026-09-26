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

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>

using srxl2::Telemetry;
using Payload = std::array<uint8_t, Telemetry::PayloadSize>;

static unsigned checks = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
			std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); std::exit(1); } } while (0)

static std::map<uint8_t, Payload> collect(Telemetry &telemetry, uint64_t now)
{
	telemetry.prepare(now);
	std::map<uint8_t, Payload> result;
	Payload payload{};
	unsigned count = 0;

	while (telemetry.nextPayload(now, payload.data())) {
		CHECK(++count <= 8);
		CHECK(result.count(payload[0]) == 0);
		result[payload[0]] = payload;
	}

	return result;
}

static Telemetry::Battery battery(uint64_t timestamp)
{
	Telemetry::Battery value{};
	value.timestamp = timestamp;
	value.connected = true;
	value.voltage_v = 22.2f;
	value.current_a = 12.34f;
	value.discharged_mah = 1234.f;
	value.temperature_c = 25.f;
	value.temperature_valid = true;
	return value;
}

static Telemetry::Gps gps(uint64_t timestamp)
{
	Telemetry::Gps value{};
	value.timestamp = timestamp;
	value.time_utc_usec = (12 * 3600 + 34 * 60 + 56) * 1000000ULL + 700000;
	value.latitude_deg = -12.5;
	value.longitude_deg = -123.25;
	value.altitude_msl_m = -1234.5;
	value.speed_m_s = 10;
	value.course_rad = 3.14159265358979323846f;
	value.hdop = 1.2f;
	value.fix_type = 3;
	value.satellites = 12;
	value.velocity_valid = true;
	return value;
}

// Numeric sensor fixtures follow the normative third-party big-endian rule in
// Spektrum X-Bus Telemetry Developers' Specifications, Rev L, section 8.
// GPS fixtures use the native packed-BCD wire format. The public-domain field
// definitions are pinned in Telemetry.hpp. ArduPilot differs for 0x34/0x1B;
// the stock NX7e+ hardware acceptance test must verify these chosen formats.
static void batteryAndUnavailable()
{
	Telemetry telemetry;
	auto frames = collect(telemetry, 1000000);
	CHECK(frames.size() == 2);
	CHECK(frames.count(0x7e) == 1);
	CHECK(frames.count(0x7f) == 1);
	const Payload qos = {0x7f, 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
			     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
			    };
	CHECK(frames.at(0x7f) == qos);

	telemetry.setBattery(battery(2000000));
	frames = collect(telemetry, 2000000);
	const Payload rpm = {0x7e, 0, 0xff, 0xff, 0x08, 0xac, 0, 0x4d,
			     0, 0, 0xff, 0xff, 0xff, 0xff, 0, 0
			    };
	const Payload capacity = {0x34, 0, 0, 0x7b, 0x04, 0xd2, 0, 0xfa,
				  0x7f, 0xff, 0x7f, 0xff, 0x7f, 0xff, 0xff, 0xff
				 };
	CHECK(frames.at(0x7e) == rpm);
	CHECK(frames.at(0x34) == capacity);

	frames = collect(telemetry, 4000001);
	CHECK(frames.at(0x7e)[4] == 0xff && frames.at(0x7e)[5] == 0xff);
	CHECK(frames.at(0x7e)[6] == 0x7f && frames.at(0x7e)[7] == 0xff);
	CHECK(frames.at(0x34)[2] == 0x7f && frames.at(0x34)[3] == 0xff);

	auto disconnected = battery(5000000);
	disconnected.connected = false;
	telemetry.setBattery(disconnected);
	frames = collect(telemetry, 5000000);
	CHECK(frames.at(0x7e)[4] == 0xff);

	auto invalid = battery(6000000);
	invalid.voltage_v = NAN;
	invalid.current_a = -1;
	invalid.discharged_mah = -1;
	invalid.temperature_c = NAN;
	telemetry.setBattery(invalid);
	frames = collect(telemetry, 6000000);
	CHECK(frames.at(0x7e)[4] == 0xff && frames.at(0x7e)[6] == 0x7f);
	CHECK(frames.at(0x34)[2] == 0x7f && frames.at(0x34)[4] == 0x7f);

	auto large = battery(7000000);
	large.voltage_v = 1e9f;
	large.current_a = 1e9f;
	large.discharged_mah = 1e9f;
	large.temperature_c = -40;
	telemetry.setBattery(large);
	frames = collect(telemetry, 7000000);
	CHECK(frames.at(0x7e)[4] == 0xff && frames.at(0x7e)[5] == 0xfe);
	CHECK(frames.at(0x34)[2] == 0x7f && frames.at(0x34)[3] == 0xfe);
	CHECK(frames.at(0x34)[4] == 0x7f && frames.at(0x34)[5] == 0xfe);
	CHECK(frames.at(0x7e)[6] == 0xff && frames.at(0x7e)[7] == 0xd8); // -40 F.
	CHECK(frames.at(0x34)[6] == 0x7f && frames.at(0x34)[7] == 0xff);
}

static void gpsEncodingAndPairing()
{
	Telemetry telemetry;
	telemetry.setGps(gps(1000000));
	auto frames = collect(telemetry, 1000000);
	const Payload location = {0x16, 0, 0x45, 0x23, 0, 0, 0x30, 0x12,
				  0, 0, 0x15, 0x23, 0, 0x18, 0x12, 0xbc
				 };
	const Payload stats = {0x17, 0, 0x94, 0x01, 0x67, 0x45, 0x23, 0x01,
			       0x12, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
			      };
	CHECK(frames.at(0x16) == location);
	CHECK(frames.at(0x17) == stats);

	// Updating uORB between the two polls must not mix GPS snapshots.
	telemetry.prepare(2000000);
	Payload packet{};
	unsigned count = 0;

	do {
		CHECK(++count <= 8);
		CHECK(telemetry.nextPayload(2000000, packet.data()));
	} while (packet[0] != 0x16);

	auto changed = gps(2000000);
	changed.satellites = 30;
	telemetry.setGps(changed);
	telemetry.prepare(2000000);
	CHECK(telemetry.nextPayload(2000000, packet.data()));
	CHECK(packet == stats);

	// Crossing a degree boundary must carry rounded minutes and the >99 longitude flag.
	changed.timestamp = 3000000;
	changed.latitude_deg = 12.999999999;
	changed.longitude_deg = 99.999999999;
	changed.altitude_msl_m = 99999.9;
	changed.time_utc_usec += 86400ULL * 1000000;
	telemetry.setGps(changed);
	frames = collect(telemetry, 3000000);
	CHECK(frames.at(0x16)[7] == 0x13);
	CHECK(frames.at(0x16)[8] == 0 && frames.at(0x16)[11] == 0);
	CHECK((frames.at(0x16)[15] & 4) != 0);
	CHECK(frames.at(0x16)[2] == 0x99 && frames.at(0x16)[3] == 0x99);
	CHECK(frames.at(0x17)[9] == 0x99);
	CHECK(frames.at(0x17)[4] == 0x67 && frames.at(0x17)[7] == 0x01);

	changed.timestamp = 4000000;
	changed.fix_type = 1;
	changed.velocity_valid = false;
	changed.hdop = NAN;
	telemetry.setGps(changed);
	frames = collect(telemetry, 4000000);
	CHECK(frames.at(0x16)[15] == 0x10);
	CHECK(frames.at(0x16)[4] == 0xff && frames.at(0x16)[12] == 0xff);
	CHECK(frames.at(0x16)[14] == 0xff && frames.at(0x17)[2] == 0xff);

	frames = collect(telemetry, 6000001);
	CHECK(frames.at(0x16)[15] == 0);
	CHECK(frames.at(0x17)[8] == 0xff);

	changed.timestamp = 7000000;
	changed.fix_type = 3;
	changed.latitude_deg = NAN;
	changed.longitude_deg = 181;
	telemetry.setGps(changed);
	frames = collect(telemetry, 7000000);
	CHECK(frames.at(0x16)[15] == 0x10);
	CHECK(frames.at(0x16)[4] == 0xff);
}

static void varioHistoryAndResets()
{
	Telemetry telemetry;
	Telemetry::Position position{};
	position.altitude_valid = true;
	position.vertical_valid = true;

	for (unsigned i = 0; i <= 60; ++i) {
		position.timestamp = 1000000 + i * 50000;
		const float seconds = i * 0.05f;
		position.local_altitude_m = seconds * seconds;
		position.altitude_m = 10 + position.local_altitude_m;
		telemetry.setPosition(position);
	}

	auto frames = collect(telemetry, 4000000);
	const Payload vario = {0x40, 0, 0, 190, 0, 58, 0, 55, 0, 50, 0, 45, 0, 40, 0, 30};
	CHECK(frames.at(0x40) == vario);

	// Changing home altitude does not produce a climb spike: history is in the local estimator frame.
	position.timestamp += 50000;
	position.local_altitude_m = 9.3025f;
	position.altitude_m = 1009.3025f;
	telemetry.setPosition(position);
	frames = collect(telemetry, 4200000);
	CHECK(frames.at(0x40)[4] == 0 && frames.at(0x40)[5] == 59);

	// An estimator reset discards old history, instead of reporting a huge vertical speed.
	position.timestamp += 50000;
	position.reset_counter++;
	position.local_altitude_m += 100;
	telemetry.setPosition(position);
	frames = collect(telemetry, 4400000);

	for (unsigned offset = 4; offset < 16; offset += 2) {
		CHECK(frames.at(0x40)[offset] == 0x7f && frames.at(0x40)[offset + 1] == 0xff);
	}

	frames = collect(telemetry, 6000000);
	CHECK(frames.at(0x40)[2] == 0x7f);

	Telemetry sink;

	for (unsigned i = 0; i <= 70; ++i) {
		position.timestamp = 1000000 + i * 50000;
		position.local_altitude_m = -static_cast<float>(i) * 0.1f;
		position.altitude_m = position.local_altitude_m;
		position.reset_counter = 0;
		sink.setPosition(position);
	}

	frames = collect(sink, position.timestamp);
	CHECK(frames.at(0x40)[2] == 0xff && frames.at(0x40)[3] == 0xba); // -7 m.

	for (unsigned offset = 4; offset < 16; offset += 2) {
		CHECK(frames.at(0x40)[offset] == 0xff && frames.at(0x40)[offset + 1] == 0xec); // -2 m/s.
	}

	position.timestamp += 300000; // A source outage invalidates all averaging windows.
	sink.setPosition(position);
	frames = collect(sink, position.timestamp);
	CHECK(frames.at(0x40)[4] == 0x7f);
}

static void attitudeAndExpiry()
{
	Telemetry telemetry;
	Telemetry::Attitude attitude{};
	attitude.timestamp = 1000000;
	attitude.valid = true;
	attitude.roll_rad = 1.5707963267948966f;
	attitude.pitch_rad = -0.7853981633974483f;
	attitude.yaw_rad = -3.14159265358979323846f;
	telemetry.setAttitude(attitude);
	auto frames = collect(telemetry, 1000000);
	const Payload expected = {0x1b, 0, 0x03, 0x84, 0xfe, 0x3e, 0xf8, 0xf8,
				  0x7f, 0xff, 0x7f, 0xff, 0x7f, 0xff, 0xff, 0xff
				 };
	CHECK(frames.at(0x1b) == expected);
	frames = collect(telemetry, 2000001);
	CHECK(frames.at(0x1b)[2] == 0x7f && frames.at(0x1b)[3] == 0xff);
	attitude.timestamp = 3000000;
	attitude.roll_rad = NAN;
	telemetry.setAttitude(attitude);
	frames = collect(telemetry, 3000000);
	CHECK(frames.at(0x1b)[2] == 0x7f);
}

static Payload nextText(Telemetry &telemetry, uint64_t now)
{
	return collect(telemetry, now).at(0x0c);
}

static void textAndBoundedQueue()
{
	Telemetry telemetry;
	Telemetry::Status status{};
	status.timestamp = 1000000;
	std::memcpy(status.mode, "Return", 7);
	status.armed = true;
	status.failsafe = true;
	telemetry.setStatus(status);

	for (unsigned i = 0; i < 8; ++i) {
		telemetry.queueMessage(1000000 + i, 6, "Routine", 8);
	}

	telemetry.queueMessage(1000010, 2, "Urgent\ncheck\t", 13);
	CHECK(telemetry.droppedMessages() == 1);
	Payload text = nextText(telemetry, 1100000);
	CHECK(text[2] == 0 && std::memcmp(text.data() + 3, "PX4", 3) == 0);
	text = nextText(telemetry, 1200000);
	CHECK(text[2] == 1 && std::memcmp(text.data() + 3, "Return", 6) == 0);
	text = nextText(telemetry, 1300000);
	CHECK(text[2] == 2 && std::memcmp(text.data() + 3, "Armed", 5) == 0);
	text = nextText(telemetry, 1400000);
	CHECK(text[2] == 3 && std::memcmp(text.data() + 3, "FAILSAFE", 8) == 0);
	text = nextText(telemetry, 1500000);
	CHECK(text[2] == 4 && std::memcmp(text.data() + 3, "Urgent check ", 12) == 0);

	// Display rows must clear once the message expires, and stale status must not imply armed/ready.
	for (unsigned i = 0; i < 9; ++i) {
		text = nextText(telemetry, 6100000 + i * 100000);

		if (text[2] >= 4) {
			CHECK(text[3] == 0);
		}

		if (text[2] == 2) {
			CHECK(std::memcmp(text.data() + 3, "State unknown", 13) == 0);
		}
	}

	char long_text[128];
	std::memset(long_text, 'A', sizeof(long_text));
	telemetry.queueMessage(8000000, 3, long_text, sizeof(long_text));

	for (unsigned i = 0; i < 9; ++i) {
		text = nextText(telemetry, 8000000 + i * 100000);

		if (text[2] == 8) {
			CHECK(std::memcmp(text.data() + 13, "...", 3) == 0);
		}
	}
}

static void ratesAndFairness()
{
	Telemetry telemetry;
	telemetry.setBattery(battery(1000000));
	telemetry.setGps(gps(1000000));
	Telemetry::Status status{};
	status.timestamp = 1000000;
	telemetry.setStatus(status);
	Telemetry::Attitude attitude{};
	attitude.timestamp = 1000000;
	attitude.valid = true;
	telemetry.setAttitude(attitude);
	Telemetry::Position position{};
	position.timestamp = 1000000;
	position.altitude_valid = true;
	telemetry.setPosition(position);
	std::map<uint8_t, unsigned> counts;

	for (uint64_t now = 1000000; now < 2000000; now += 10000) {
		telemetry.prepare(now);
		Payload packet{};

		if (telemetry.nextPayload(now, packet.data())) {
			++counts[packet[0]];
		}
	}

	CHECK(counts.size() == 8);
	CHECK(counts[0x7e] >= 8 && counts[0x7e] <= 10);
	CHECK(counts[0x7f] >= 8 && counts[0x7f] <= 10);
	CHECK(counts[0x0c] >= 8 && counts[0x0c] <= 10);
	CHECK(counts[0x34] == 2);
	CHECK(counts[0x16] == 2 && counts[0x17] == 2);
	CHECK(counts[0x1b] >= 4 && counts[0x1b] <= 5);
	CHECK(counts[0x40] >= 4 && counts[0x40] <= 5);

	// At fewer grants than requested telemetry rates, every class still gets a turn.
	counts.clear();

	for (uint64_t now = 2000000; now < 4000000; now += 100000) {
		telemetry.prepare(now);
		Payload packet{};

		if (telemetry.nextPayload(now, packet.data())) {
			++counts[packet[0]];
		}
	}

	CHECK(counts.size() == 8);
}

int main()
{
	batteryAndUnavailable();
	gpsEncodingAndPairing();
	varioHistoryAndResets();
	attitudeAndExpiry();
	textAndBoundedQueue();
	ratesAndFairness();
	std::printf("SRXL2 telemetry: %u checks passed\n", checks);
	return 0;
}
