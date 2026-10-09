// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#include "DisplayPort.hpp"

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

using namespace msp_osd;

namespace
{
constexpr float PI = 3.14159265358979323846f;
constexpr uint32_t ALL_SYMBOLS = (1u << (AIRSPEED_SP + 1)) - 1;

struct Capture {
	std::vector<std::vector<uint8_t>> packets;
	std::array<std::array<uint16_t, 60>, 22> screen{};
	int fail_after{-1};
	bool valid{true};
	unsigned columns{60};
	unsigned rows{22};

	static bool write(void *context, const uint8_t *payload, size_t size)
	{
		auto &capture = *static_cast<Capture *>(context);

		if (capture.fail_after >= 0 && capture.packets.size() >= static_cast<size_t>(capture.fail_after)) {
			return false;
		}

		capture.packets.emplace_back(payload, payload + size);

		if (size == 1 && payload[0] == 2) {
			for (auto &row : capture.screen) { row.fill(' '); }

		} else if (payload[0] == 3) {
			if (size < 6 || size > 35 || payload[size - 1] != 0 || payload[3] > 1
			    || payload[1] >= capture.rows || payload[2] + size - 5 > capture.columns) {
				capture.valid = false;
				return false;
			}

			for (size_t i = 4; i < size - 1; ++i) {
				capture.screen[payload[1]][payload[2] + i - 4] = payload[i] | (payload[3] << 8);
			}
		}

		return true;
	}

	std::string line(unsigned row) const
	{
		return std::string(screen[row].begin(), screen[row].end());
	}

	size_t wire_bytes() const
	{
		size_t bytes = 0;

		for (const auto &packet : packets) { bytes += packet.size() + 6; }

		return bytes;
	}

	size_t writes() const
	{
		size_t count = 0;

		for (const auto &packet : packets) { count += packet[0] == 3; }

		return count;
	}

	int first_horizon_row(int column) const
	{
		for (int row = 0; row < 22; ++row) {
			const uint16_t value = screen[row][column];

			if ((value >= 0x80 && value <= 0x88) || (value >= 0x14c && value <= 0x154)) { return row; }
		}

		return -1;
	}
};

void attitude(OsdData &data, float roll_deg, float pitch_deg = 0.f, float yaw_deg = 0.f)
{
	const float cr = cosf(roll_deg * PI / 360.f);
	const float sr = sinf(roll_deg * PI / 360.f);
	const float cp = cosf(pitch_deg * PI / 360.f);
	const float sp = sinf(pitch_deg * PI / 360.f);
	const float cy = cosf(yaw_deg * PI / 360.f);
	const float sy = sinf(yaw_deg * PI / 360.f);
	data.attitude_q[0] = cr * cp * cy + sr * sp * sy;
	data.attitude_q[1] = sr * cp * cy - cr * sp * sy;
	data.attitude_q[2] = cr * sp * cy + sr * cp * sy;
	data.attitude_q[3] = cr * cp * sy - sr * sp * cy;
}

OsdData sample()
{
	OsdData data;
	data.battery_voltage = 24.4f;
	data.cell_voltage = 4.07f;
	data.compensated_battery_voltage = 25.2f;
	data.compensated_cell_voltage = 4.2f;
	data.battery_remaining_percent = 42.f;
	data.current_a = 12.3f;
	data.discharged_mah = 1234.f;
	data.ground_speed_m_s = 10.f;
	data.airspeed_m_s = 20.f;
	data.airspeed_setpoint_m_s = 22.f;
	data.altitude_m = 100.f;
	data.vertical_speed_m_s = -1.f;
	data.home_distance_m = 1000.f;
	data.home_bearing_rad = 0.f;
	attitude(data, 0.f);
	data.roll_rad = 0.f;
	data.pitch_rad = 0.f;
	data.throttle_percent = 50.f;
	data.rssi_percent = 99.f;
	data.latitude_deg = 32.12345;
	data.longitude_deg = -112.12345;
	data.satellites = 18;
	data.status_valid = true;
	data.armed = true;
	data.flight_seconds = 65;
	data.flight_time_valid = true;
	strcpy(data.mode, "Position FW");
	strcpy(data.message, "Autotune: roll");
	return data;
}
}

TEST(DisplayPort, AllRequestedItemsAndUnits)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.symbols = ALL_SYMBOLS;
	ASSERT_TRUE(display.render(sample(), settings));
	EXPECT_TRUE(capture.valid);
	EXPECT_EQ(capture.line(1).substr(1, 11), "POSITION FW");
	EXPECT_EQ(capture.line(1).substr(18, 3), "ARM");
	EXPECT_EQ(capture.screen[1][22], 0x11);
	EXPECT_EQ(capture.screen[1][23], 0x68);
	EXPECT_EQ(capture.line(1).substr(25, 5), "3281\x0f");
	EXPECT_EQ(capture.line(1).substr(42, 7), "\x9c 01:05");
	EXPECT_EQ(capture.line(3).substr(1, 14), "AUTOTUNE: ROLL");
	EXPECT_EQ(capture.line(9).substr(11, 7), "\x70 22.4\x9d");
	EXPECT_EQ(capture.line(7).substr(11, 8), std::string(8, ' '));
	EXPECT_EQ(capture.line(8).substr(11, 8), std::string(8, ' '));
	EXPECT_EQ(capture.line(6).substr(11, 8), "AS 44.7\x9d");
	EXPECT_EQ(capture.line(6).substr(34, 4), "328\x0f");
	EXPECT_EQ(capture.line(19).substr(1, 5), "\x04 50%");
	EXPECT_EQ(capture.line(19).substr(20, 13), "C 25.2V/4.20V");
	EXPECT_EQ(capture.line(17).substr(21, 11), "24.4V/4.07V");
	EXPECT_EQ(capture.line(18).substr(16, 21), "\x93 42%   1234\x07   12.3\x9a");
	settings.imperial = false;
	ASSERT_TRUE(display.render(sample(), settings));
	EXPECT_EQ(capture.line(9).substr(11, 7), "\x70 10.0\x9f");
	EXPECT_EQ(capture.line(6).substr(11, 8), "AS 20.0\x9f");
	EXPECT_EQ(capture.line(6).substr(34, 4), "100\x0c");
}

TEST(DisplayPort, MaskBitsAreIndependent)
{
	const OsdData data = sample();

	for (unsigned bit = 0; bit <= AIRSPEED_SP; ++bit) {
		Capture capture;
		DisplayPort display(Capture::write, &capture);
		DisplaySettings settings;
		settings.columns = 60;
		settings.rows = 22;
		settings.symbols = 1u << bit;
		ASSERT_TRUE(display.render(data, settings)) << bit;
		ASSERT_TRUE(capture.valid) << bit;

		if (bit == ESC_TMP || bit == CRAFT_NAME) {
			EXPECT_EQ(capture.writes(), 0u);

		} else {
			EXPECT_GT(capture.writes(), 0u) << bit;
		}

		if (bit != MESSAGES) { EXPECT_EQ(capture.line(3).find("AUTOTUNE"), std::string::npos) << bit; }

		if (bit != CURRENT_DRAW) { EXPECT_EQ(capture.line(18).find("12.3\x9a"), std::string::npos) << bit; }

		if (bit != FLYMODE) { EXPECT_EQ(capture.line(1).find("POSITION"), std::string::npos) << bit; }

		if (bit == MAIN_BATT_VOLTAGE) { EXPECT_EQ(capture.line(17).substr(21, 5), "24.4V"); }

		if (bit == AVG_CELL_VOLTAGE) { EXPECT_EQ(capture.line(17).substr(27, 5), "4.07V"); }

		if (bit == BATT_COMP_VOLTAGE) { EXPECT_NE(capture.line(19).find("C 25.2V"), std::string::npos); }

		if (bit == BATT_CELL_COMP_VOLTAGE) { EXPECT_NE(capture.line(19).find("C 4.20V"), std::string::npos); }

		if (bit == BATT_PERC) { EXPECT_NE(capture.line(18).find("\x93 42%"), std::string::npos); }
	}
}

TEST(DisplayPort, FrameOrderingAndDisableRelease)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.columns = 60;
	settings.rows = 22;
	settings.symbols = 1u << FLIGHT_TIME;
	ASSERT_TRUE(display.render(sample(), settings));
	ASSERT_EQ(capture.packets.size(), 5u);
	EXPECT_EQ(capture.packets[0][0], 0);
	EXPECT_EQ(capture.packets[1], (std::vector<uint8_t> {5, 0, 3}));
	EXPECT_EQ(capture.packets[2][0], 2);
	EXPECT_EQ(capture.packets[3][0], 3);
	EXPECT_EQ(capture.packets[4][0], 4);
	capture.packets.clear();
	settings.symbols = 0;
	ASSERT_TRUE(display.render(sample(), settings));
	ASSERT_EQ(capture.packets.size(), 3u);
	EXPECT_EQ(capture.packets[0][0], 2);
	EXPECT_EQ(capture.packets[1][0], 4);
	EXPECT_EQ(capture.packets[2][0], 1);
	EXPECT_EQ(capture.line(1), std::string(60, ' '));
}

TEST(DisplayPort, StopsOnTransportFailureWithoutDrawingPartialFrame)
{
	Capture capture;
	capture.fail_after = 3;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.columns = 60;
	settings.rows = 22;
	settings.symbols = ALL_SYMBOLS;
	EXPECT_FALSE(display.render(sample(), settings));
	ASSERT_EQ(capture.packets.size(), 3u);
	EXPECT_NE(capture.packets.back()[0], 4);
	capture.fail_after = -1;
	capture.packets.clear();
	ASSERT_TRUE(display.render(sample(), settings));
	EXPECT_EQ(capture.packets.front()[0], 0);
	EXPECT_EQ(capture.packets.back()[0], 4);
}

TEST(DisplayPort, CompactCanvasAcceptsExplicitPositions)
{
	Capture capture;
	capture.columns = 30;
	capture.rows = 16;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.columns = 30;
	settings.rows = 16;
	settings.symbols = (1u << AIRSPEED) | (1u << GPS_SPEED) | (1u << ALTITUDE) | (1u << FLIGHT_TIME)
			   | (1u << BATT_PERC) | (1u << MAH_DRAWN) | (1u << CURRENT_DRAW) | (1u << THROTTLE);
	settings.positions[AIRSPEED] = {0, 5};
	settings.positions[GPS_SPEED] = {0, 8};
	settings.positions[ALTITUDE] = {23, 5};
	settings.positions[FLIGHT_TIME] = {19, 1};
	settings.positions[BATT_PERC] = {2, 12};
	settings.positions[MAH_DRAWN] = {11, 12};
	settings.positions[CURRENT_DRAW] = {21, 12};
	settings.positions[THROTTLE] = {0, 15};
	ASSERT_TRUE(display.render(sample(), settings));
	ASSERT_TRUE(capture.valid);
	EXPECT_EQ(capture.line(1).substr(19, 7), "\x9c 01:05");
	EXPECT_EQ(capture.line(5).substr(0, 8), "AS 44.7\x9d");
	EXPECT_EQ(capture.line(5).substr(23, 4), "328\x0f");
	EXPECT_EQ(capture.line(8).substr(0, 7), "\x70 22.4\x9d");
	EXPECT_EQ(capture.line(12).substr(2, 5), "\x93 42%");
	EXPECT_EQ(capture.line(12).substr(11, 5), "1234\x07");
	EXPECT_EQ(capture.line(12).substr(21, 5), "12.3\x9a");
	EXPECT_EQ(capture.line(15).substr(0, 5), "\x04 50%");
}

TEST(DisplayPort, InvalidCanvasAndExtremePositionsAreBounded)
{
	for (uint8_t size : {0, 1, 15, 29, 255}) {
		Capture capture;
		DisplayPort display(Capture::write, &capture);
		DisplaySettings settings;
		settings.columns = 60;
		settings.rows = 22;
		settings.columns = size;
		settings.rows = size;
		settings.positions[CROSSHAIRS] = {59, 21};
		settings.symbols = ALL_SYMBOLS;
		ASSERT_TRUE(display.render(sample(), settings));
		EXPECT_TRUE(capture.valid);
	}
}

TEST(DisplayPort, InvalidNumbersShowUnavailable)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.symbols = ALL_SYMBOLS;
	OsdData data;
	data.current_a = -1.f;
	data.discharged_mah = INFINITY;
	data.ground_speed_m_s = std::numeric_limits<float>::max();
	data.battery_voltage = -1.f;
	data.throttle_percent = 101.f;
	data.latitude_deg = 91.0;
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.line(9).substr(11, 6), "\x70 0.0\x9d");
	EXPECT_EQ(capture.line(18).substr(24, 3), "--\x07");
	EXPECT_EQ(capture.line(18).substr(32, 3), "--\x9a");
	EXPECT_EQ(capture.line(19).substr(1, 5), "\x04 --%");
	EXPECT_EQ(capture.line(17).substr(21, 3), "--V");
	EXPECT_EQ(capture.screen[17][26], '/');
	EXPECT_EQ(capture.line(17).substr(27, 3), "--V");
	EXPECT_EQ(capture.line(15).substr(1, 6), "LAT --");
	EXPECT_NE(capture.line(10).find("HORIZON --"), std::string::npos);

	for (unsigned row = 0; row < 22; ++row) {
		EXPECT_EQ(capture.line(row).find("NAN"), std::string::npos);
		EXPECT_EQ(capture.line(row).find("INF"), std::string::npos);
	}
}

TEST(DisplayPort, NumericOverflowDoesNotTruncateToMisleadingValue)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.columns = 60;
	settings.rows = 22;
	settings.symbols = (1u << MAH_DRAWN) | (1u << CURRENT_DRAW);
	OsdData data = sample();
	data.discharged_mah = 1e9f;
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.line(18).substr(24, 13), "--\x07     12.3\x9a");
}

TEST(DisplayPort, FlightTimerBoundaries)
{
	const uint32_t seconds[] = {0, 59, 60, 3599, 3600, 359999, 360000, UINT32_MAX};
	const char *expected[] = {"\x9c 00:00", "\x9c 00:59", "\x9c 01:00", "\x9c 59:59", "\x9c 01:00:00", "\x9c 99:59:59", "\x9c >99H", "\x9c >99H"};
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.columns = 60;
	settings.rows = 22;
	settings.symbols = 1u << FLIGHT_TIME;
	OsdData data = sample();

	for (unsigned i = 0; i < sizeof(seconds) / sizeof(seconds[0]); ++i) {
		data.flight_seconds = seconds[i];
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.line(1).substr(42, strlen(expected[i])), expected[i]);
	}
}

TEST(DisplayPort, TextIsBoundedAndCompatibleWithBetaflightGlyphs)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.columns = 60;
	settings.rows = 22;
	settings.symbols = (1u << MESSAGES) | (1u << FLYMODE) | (1u << AIRSPEED);
	OsdData data = sample();
	memset(data.message, 'a', sizeof(data.message)); // Deliberately unterminated input.
	memset(data.mode, 'b', sizeof(data.mode));
	data.airspeed_estimated = true;
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_TRUE(capture.valid);
	EXPECT_EQ(capture.line(3).substr(1, 30), std::string(30, 'A'));
	EXPECT_EQ(capture.line(1).substr(1, 16), std::string(16, 'B'));
	EXPECT_EQ(capture.line(6).substr(13, 1), "*");
	data.message[0] = '\n';
	data.message[1] = static_cast<char>(0x90);
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.line(3).substr(1, 2), "??");
	settings.inav_font = true;
	strcpy(data.message, "test \"$'?\n");
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.line(3).substr(1, 10), "TEST -----");
}

TEST(DisplayPort, HomeArrowUsesCameraHeadingAndWraps)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.columns = 60;
	settings.rows = 22;
	settings.symbols = 1u << HOME_DIR;
	OsdData data = sample();
	const float bearings[] = {0.f, PI / 2.f, PI, -PI / 2.f, 2.f * PI};
	const uint8_t glyphs[] = {0x68, 0x64, 0x60, 0x6c, 0x68};

	for (unsigned i = 0; i < sizeof(bearings) / sizeof(bearings[0]); ++i) {
		data.home_bearing_rad = bearings[i];
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.screen[1][23], glyphs[i]);
	}

	attitude(data, 0.f, 0.f, 90.f);
	data.home_bearing_rad = PI / 2.f;
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.screen[1][23], 0x68);
	attitude(data, 0.f, 90.f);
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.screen[1][23], 0x68);
	settings.camera_pitch_deg = -90.f;
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.screen[1][23], 0x64);
}

TEST(DisplayPort, HomeArrowFallsBackForwardAndRecoversInBothFonts)
{
	for (bool inav : {false, true}) {
		Capture capture;
		DisplayPort display(Capture::write, &capture);
		DisplaySettings settings;
		settings.symbols = 1u << HOME_DIR;
		settings.inav_font = inav;
		const uint16_t forward = inav ? 0x13c : 0x68;
		OsdData data = sample();
		attitude(data, 0.f, 0.f, 90.f); // Fallback follows the screen, independent of aircraft yaw.

		for (float bearing : {NAN, INFINITY, -INFINITY}) {
			data.home_bearing_rad = bearing;
			ASSERT_TRUE(display.render(data, settings));
			EXPECT_EQ(capture.screen[1][23], forward);
		}

		data.home_bearing_rad = PI / 2.f;
		attitude(data, 0.f, 90.f); // Camera heading is undefined when looking vertically.
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.screen[1][23], forward);
		memset(data.attitude_q, 0, sizeof(data.attitude_q));
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.screen[1][23], forward);
		data.attitude_q[0] = NAN;
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.screen[1][23], forward);

		attitude(data, 0.f);
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.screen[1][23], inav ? 0x140 : 0x64); // Valid rightward home bearing resumes.
		EXPECT_TRUE(capture.valid);
	}
}

TEST(DisplayPort, HorizonLevelRollAndPitchDirection)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.columns = 60;
	settings.rows = 22;
	settings.symbols = 1u << ARTIFICIAL_HORIZON;
	settings.positions[ARTIFICIAL_HORIZON] = {30, 8};
	OsdData data = sample();
	ASSERT_TRUE(display.render(data, settings));

	for (int column = 25; column <= 35; ++column) { EXPECT_EQ(capture.first_horizon_row(column), 8); }

	EXPECT_EQ(capture.writes(), 1u); // Contiguous horizon strokes share one packet.
	attitude(data, 30.f);
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_GT(capture.first_horizon_row(26), capture.first_horizon_row(34));
	attitude(data, -30.f);
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_LT(capture.first_horizon_row(26), capture.first_horizon_row(34));
	attitude(data, 0.f, 5.f);
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_GT(capture.first_horizon_row(30), 8); // Nose up moves visible horizon down.
	attitude(data, 0.f, -5.f);
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_LT(capture.first_horizon_row(30), 8);
}

TEST(DisplayPort, HorizonVerticalInvertedAndOffscreen)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.columns = 60;
	settings.rows = 22;
	settings.symbols = 1u << ARTIFICIAL_HORIZON;
	settings.positions[ARTIFICIAL_HORIZON] = {30, 8};
	OsdData data = sample();

	for (float roll : {-90.f, 90.f}) {
		attitude(data, roll);
		ASSERT_TRUE(display.render(data, settings));

		for (int row = 5; row <= 11; ++row) { EXPECT_EQ(capture.screen[row][30], 0x13); }
	}

	attitude(data, 180.f);
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.first_horizon_row(30), 8);

	for (float pitch : {-90.f, -30.f, 30.f, 90.f}) {
		attitude(data, 0.f, pitch);
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.first_horizon_row(30), -1);
	}

	attitude(data, 0.f, 180.f);
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.first_horizon_row(30), 8);
}

TEST(DisplayPort, HorizonCameraMountAndInvalidQuaternion)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.columns = 60;
	settings.rows = 22;
	settings.symbols = 1u << ARTIFICIAL_HORIZON;
	settings.positions[ARTIFICIAL_HORIZON] = {30, 8};
	settings.camera_pitch_deg = 5.f;
	OsdData data = sample();
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_GT(capture.first_horizon_row(30), 8);
	attitude(data, 0.f, -5.f);
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.first_horizon_row(30), 8);
	memset(data.attitude_q, 0, sizeof(data.attitude_q));
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_NE(capture.line(11).find("HORIZON --"), std::string::npos);
	data.attitude_q[0] = INFINITY;
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_NE(capture.line(11).find("HORIZON --"), std::string::npos);
}

TEST(DisplayPort, FullMaskWireBudgetAndClippingAcrossAttitudes)
{
	DisplaySettings settings;
	settings.columns = 60;
	settings.rows = 22;
	settings.symbols = ALL_SYMBOLS;
	OsdData data = sample();
	memset(data.message, 'M', sizeof(data.message) - 1);

	for (float roll : {-180.f, -90.f, -30.f, 0.f, 30.f, 90.f, 180.f}) {
		for (float pitch : {-180.f, -90.f, -30.f, 0.f, 30.f, 90.f, 180.f}) {
			Capture capture;
			DisplayPort display(Capture::write, &capture);
			attitude(data, roll, pitch);
			ASSERT_TRUE(display.render(data, settings));
			EXPECT_TRUE(capture.valid);
			// At 115200 8N1, 10 Hz allows 1152 wire bytes/frame; reserve telemetry and headroom.
			EXPECT_LT(capture.wire_bytes(), 950u) << roll << ", " << pitch;
		}
	}
}

TEST(DisplayPort, InavUnitsAndExtendedGlyphPagesMatchN3Map)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.symbols = ALL_SYMBOLS;
	settings.inav_font = true;
	ASSERT_TRUE(display.render(sample(), settings));
	ASSERT_TRUE(capture.valid);
	EXPECT_EQ(capture.screen[1][22], 0x10); // home
	EXPECT_EQ(capture.screen[1][23], 0x13c); // north arrow, page 1
	EXPECT_EQ(capture.screen[1][29], 0x74); // home distance feet
	EXPECT_EQ(capture.screen[1][42], 0x9f); // flight timer
	EXPECT_EQ(capture.screen[6][11], 0x8c); // airspeed icon
	EXPECT_EQ(capture.screen[9][11], 0x17); // ground speed
	EXPECT_EQ(capture.screen[19][1], 0x95); // throttle icon
	EXPECT_EQ(capture.screen[6][17], 0x91); // mph
	EXPECT_EQ(capture.screen[6][37], 0x78); // altitude feet
	EXPECT_EQ(capture.screen[18][16], 0x66); // battery level
	EXPECT_EQ(capture.screen[18][28], 0x99); // mAh
	EXPECT_EQ(capture.screen[18][36], 0x6a); // amps
	EXPECT_EQ(capture.screen[7][21], 0x150); // horizon, page 1
	EXPECT_EQ(capture.screen[7][26], 0x166); // reticle, page 1
	settings.imperial = false;
	ASSERT_TRUE(display.render(sample(), settings));
	EXPECT_EQ(capture.screen[6][17], 0x8f); // m/s
	EXPECT_EQ(capture.screen[6][37], 0x76); // altitude meters
}

TEST(DisplayPort, InavHomeArrowsKeepAllCompassDirections)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.columns = 60;
	settings.rows = 22;
	settings.symbols = 1u << HOME_DIR;
	settings.inav_font = true;
	OsdData data = sample();

	for (int sector = 0; sector < 16; ++sector) {
		data.home_bearing_rad = sector * PI / 8.f;
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.screen[1][23], 0x13c + sector);
	}
}

TEST(DisplayPort, CapacityChangesToAmpHoursWithoutLosingUnits)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.symbols = 1u << MAH_DRAWN;
	settings.inav_font = true;
	OsdData data = sample();
	data.discharged_mah = 9999.f;
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.line(18).substr(24, 4), "9999");
	EXPECT_EQ(capture.screen[18][28], 0x99);
	data.discharged_mah = 10420.f;
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.line(18).substr(24, 5), "10.42");
	EXPECT_EQ(capture.screen[18][29], 0xd3);
	settings.inav_font = false;
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.line(18).substr(24, 7), "10.42AH"); // No standard Betaflight Ah glyph.
	settings.symbols |= (1u << MAIN_BATT_VOLTAGE) | (1u << AVG_CELL_VOLTAGE);
	data.battery_voltage = NAN;
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.line(17).substr(21, 3), "--V");
	EXPECT_EQ(capture.line(17).substr(27, 5), "4.07V");
	data.cell_voltage = NAN;
	data.discharged_mah = 1e8f;
	ASSERT_TRUE(display.render(data, settings));
	EXPECT_EQ(capture.line(17).substr(27, 3), "--V");
	EXPECT_EQ(capture.line(18).substr(24, 4), "--AH");
}

TEST(DisplayPort, CompensatedVoltageAndPercentageDoNotOverwriteOtherBatteryFields)
{
	for (bool inav : {false, true}) {
		Capture capture;
		DisplayPort display(Capture::write, &capture);
		DisplaySettings settings;
		settings.inav_font = inav;
		settings.symbols = ALL_SYMBOLS;
		OsdData data = sample();
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_TRUE(capture.valid);
		EXPECT_EQ(capture.line(17).substr(21, 11), "24.4V/4.07V");
		EXPECT_EQ(capture.line(19).substr(20, 13), "C 25.2V/4.20V");
		EXPECT_EQ(capture.line(18).substr(18, 3), "42%");
		EXPECT_EQ(capture.line(18).substr(24, 4), "1234");
		EXPECT_EQ(capture.line(18).substr(32, 4), "12.3");
		EXPECT_EQ(capture.line(17).substr(1, 4), "300W");
		settings.symbols &= ~((1u << MAIN_BATT_VOLTAGE) | (1u << AVG_CELL_VOLTAGE));
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.line(19).substr(20, 13), "C 25.2V/4.20V");
		EXPECT_EQ(capture.line(17).find("24.4V"), std::string::npos);
		data.compensated_battery_voltage = NAN;
		data.compensated_cell_voltage = NAN;
		data.battery_remaining_percent = NAN;
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.line(19).substr(20, 5), "C --V");
		EXPECT_EQ(capture.screen[19][27], '/');
		EXPECT_EQ(capture.line(19).substr(28, 3), "--V");
		EXPECT_EQ(capture.line(18).substr(18, 3), "--%");
		data.battery_remaining_percent = 0.f;
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.line(18).substr(18, 2), "0%");
	}
}

TEST(DisplayPort, BatteryPositionsRemainFixedAcrossValuesAndMasks)
{
	for (bool inav : {false, true}) {
		Capture capture;
		DisplayPort display(Capture::write, &capture);
		DisplaySettings settings;
		settings.inav_font = inav;
		settings.symbols = (1u << BATT_PERC) | (1u << MAH_DRAWN) | (1u << CURRENT_DRAW);
		OsdData data = sample();
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.screen[18][16], inav ? 0x66 : 0x93);
		EXPECT_EQ(capture.line(18).substr(24, 4), "1234");
		EXPECT_EQ(capture.line(18).substr(32, 4), "12.3");
		settings.symbols = 1u << CURRENT_DRAW;
		data.current_a = 0.f;
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.line(18).substr(32, 3), "0.0");
		EXPECT_EQ(capture.line(18).substr(0, 32), std::string(32, ' '));
		settings.symbols = 1u << BATT_PERC;
		const float percentages[] = {0.f, 50.f, 100.f, NAN};
		const int levels[] = {6, 3, 0, 6};
		const char *labels[] = {" 0%", " 50%", " 100%", " --%"};

		for (unsigned i = 0; i < 4; ++i) {
			data.battery_remaining_percent = percentages[i];
			ASSERT_TRUE(display.render(data, settings));
			EXPECT_EQ(capture.screen[18][16], (inav ? 0x63 : 0x90) + levels[i]);
			EXPECT_EQ(capture.line(18).substr(17, strlen(labels[i])), labels[i]);
		}
	}
}

TEST(DisplayPort, CanvasChangesClipWithoutRepositioningItems)
{
	for (const auto canvas : {std::array<unsigned, 2> {60, 22}, {53, 20}, {50, 18}, {30, 16}}) {
		Capture capture;
		capture.columns = canvas[0];
		capture.rows = canvas[1];
		DisplayPort display(Capture::write, &capture);
		DisplaySettings settings;
		settings.columns = canvas[0];
		settings.rows = canvas[1];
		settings.symbols = (1u << FLYMODE) | (1u << DISARMED) | (1u << FLIGHT_TIME) | (1u << CURRENT_DRAW);
		OsdData data = sample();
		memset(data.mode, 'M', sizeof(data.mode));
		data.armed = false;
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_TRUE(capture.valid);
		EXPECT_EQ(capture.line(1).substr(1, 16), std::string(16, 'M'));
		EXPECT_EQ(capture.line(1).substr(18, 3), "DIS");
		EXPECT_EQ(capture.line(1).find("PX4"), std::string::npos);
		EXPECT_EQ(capture.screen[1][42], canvas[0] > 42 ? 0x9c : ' ');
		EXPECT_EQ(capture.line(18).substr(32, 4), canvas[1] > 18 ? "12.3" : "    ");
		data.armed = true;
		data.failsafe = true;
		ASSERT_TRUE(display.render(data, settings));
		EXPECT_EQ(capture.line(1).substr(18, 4), "ARM!");
	}
}

TEST(DisplayPort, DjiCanvasDefaultsFitAndLeaveTopRowClear)
{
	Capture capture;
	capture.columns = 53;
	capture.rows = 20;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.symbols = 1070882546u;
	settings.inav_font = true;
	ASSERT_EQ(settings.columns, 53);
	ASSERT_EQ(settings.rows, 20);
	ASSERT_TRUE(display.render(sample(), settings));
	EXPECT_TRUE(capture.valid);
	EXPECT_EQ(capture.line(0), std::string(60, ' '));
	EXPECT_EQ(capture.line(1).substr(1, 11), "POSITION FW");
	EXPECT_EQ(capture.line(1).substr(18, 3), "ARM");
	EXPECT_EQ(capture.screen[1][22], 0x10); // Home icon.
	EXPECT_EQ(capture.screen[1][48], '5'); // Flight timer stays inside the right edge.
	EXPECT_NE(capture.line(3).find("AUTOTUNE: ROLL"), std::string::npos);
	EXPECT_EQ(capture.screen[7][26], 0x166); // Reticle at the 53-column centre.
	EXPECT_EQ(capture.screen[7][19], 0x131); // Left sidebar.
	EXPECT_EQ(capture.screen[7][33], 0x131); // Right sidebar.
	EXPECT_NE(capture.line(18).find("42%"), std::string::npos);
	EXPECT_NE(capture.line(18).find("SAT 18"), std::string::npos);
	EXPECT_NE(capture.line(19).find("C 25.2V/4.20V"), std::string::npos);
	EXPECT_NE(capture.line(19).find("RC 99%"), std::string::npos);
	EXPECT_EQ(capture.screen[19][1], 0x95); // Throttle reaches only the last valid row.

	for (const auto &packet : capture.packets) {
		EXPECT_NE(packet[0], 5); // No legacy resolution option can override the DJI grid.
	}
}

TEST(DisplayPort, MissingAndInvalidSpeedsDisplayZeroWithoutChangingOtherFields)
{
	for (bool imperial : {false, true}) {
		for (bool inav : {false, true}) {
			Capture capture;
			DisplayPort display(Capture::write, &capture);
			DisplaySettings settings;
			settings.symbols = (1u << AIRSPEED) | (1u << GPS_SPEED) | (1u << ALTITUDE);
			settings.imperial = imperial;
			settings.inav_font = inav;
			OsdData data;

			for (float value : {0.f, -0.f, -1.f, NAN, INFINITY, -INFINITY, std::numeric_limits<float>::max()}) {
				data.airspeed_m_s = data.ground_speed_m_s = value;
				ASSERT_TRUE(display.render(data, settings));
				EXPECT_NE(capture.line(6).substr(9, 10).find("0.0"), std::string::npos);
				EXPECT_NE(capture.line(9).substr(9, 10).find("0.0"), std::string::npos);
				EXPECT_EQ(capture.line(6).substr(9, 10).find("--"), std::string::npos);
				EXPECT_EQ(capture.line(9).substr(9, 10).find("--"), std::string::npos);
				EXPECT_NE(capture.line(6).substr(34).find("--"), std::string::npos); // Altitude still unavailable.
			}

			data.airspeed_m_s = data.ground_speed_m_s = 0.5f;
			ASSERT_TRUE(display.render(data, settings));
			EXPECT_NE(capture.line(6).substr(9, 10).find(imperial ? "1.1" : "0.5"), std::string::npos);
			EXPECT_NE(capture.line(9).substr(9, 10).find(imperial ? "1.1" : "0.5"), std::string::npos);
		}
	}
}

TEST(DisplayPort, EveryImplementedItemHasIndependentXYPlacement)
{
	for (unsigned bit = 1; bit <= AIRSPEED_SP; ++bit) {
		if (bit == ESC_TMP) { continue; }

		Capture capture;
		capture.columns = 53;
		capture.rows = 20;
		DisplayPort display(Capture::write, &capture);
		DisplaySettings settings;
		settings.symbols = 1u << bit;
		settings.positions[bit] = {10, 12};
		ASSERT_TRUE(display.render(sample(), settings)) << bit;
		ASSERT_TRUE(capture.valid) << bit;
		int minimum_x = 60;
		int minimum_y = 22;

		for (const auto &packet : capture.packets) {
			if (packet[0] != 3) { continue; }

			minimum_x = std::min(minimum_x, static_cast<int>(packet[2]));
			minimum_y = std::min(minimum_y, static_cast<int>(packet[1]));
		}

		const int left_extent = bit == HORIZON_SIDEBARS ? 7 : bit == ARTIFICIAL_HORIZON ? 5 : bit == CROSSHAIRS ? 1 : 0;
		EXPECT_EQ(minimum_x, 10 - left_extent) << bit;
		EXPECT_EQ(minimum_y, bit == HORIZON_SIDEBARS ? 9 : 12) << bit;
	}
}

TEST(DisplayPort, LivePositionUpdatesAndExplicitDefaultsClearOldLocations)
{
	Capture capture;
	DisplayPort display(Capture::write, &capture);
	DisplaySettings settings;
	settings.symbols = (1u << AIRSPEED) | (1u << CURRENT_DRAW) | (1u << BATT_COMP_VOLTAGE) | (1u << BATT_CELL_COMP_VOLTAGE);
	settings.positions[AIRSPEED] = {2, 6};
	settings.positions[CURRENT_DRAW] = {5, 4};
	settings.positions[BATT_COMP_VOLTAGE] = {1, 15};
	settings.positions[BATT_CELL_COMP_VOLTAGE] = {40, 16};
	ASSERT_TRUE(display.render(sample(), settings));
	EXPECT_EQ(capture.line(6).substr(2, 8), "AS 44.7\x9d");
	EXPECT_EQ(capture.line(4).substr(5, 5), "12.3\x9a");
	EXPECT_EQ(capture.line(15).substr(1, 7), "C 25.2V");
	EXPECT_EQ(capture.line(16).substr(40, 7), "C 4.20V");
	const DisplaySettings defaults;

	for (auto item : {AIRSPEED, CURRENT_DRAW, BATT_COMP_VOLTAGE, BATT_CELL_COMP_VOLTAGE}) {
		settings.positions[item] = defaults.positions[item];
	}
	ASSERT_TRUE(display.render(sample(), settings));
	EXPECT_EQ(capture.line(4), std::string(60, ' '));
	EXPECT_EQ(capture.line(6).substr(11, 8), "AS 44.7\x9d");
	EXPECT_EQ(capture.line(18).substr(32, 5), "12.3\x9a");
	EXPECT_EQ(capture.line(19).substr(20, 13), "C 25.2V/4.20V");
}

TEST(DisplayPort, OffscreenPositionsClipWithoutAbortingOtherItems)
{
	for (const auto canvas : {std::array<unsigned, 2> {53, 20}, {30, 16}}) {
		for (unsigned bit = 1; bit <= AIRSPEED_SP; ++bit) {
			if (bit == ESC_TMP || bit == FLIGHT_TIME) { continue; }

			Capture capture;
			capture.columns = canvas[0];
			capture.rows = canvas[1];
			DisplayPort display(Capture::write, &capture);
			DisplaySettings settings;
			settings.columns = canvas[0];
			settings.rows = canvas[1];
			settings.symbols = (1u << bit) | (1u << FLIGHT_TIME);
			settings.positions[bit] = {59, 21};
			settings.positions[FLIGHT_TIME] = {1, 1};
			ASSERT_TRUE(display.render(sample(), settings)) << bit;
			ASSERT_TRUE(capture.valid) << bit;
			EXPECT_NE(capture.line(1).find("01:05"), std::string::npos) << bit;
			EXPECT_EQ(capture.packets.back()[0], 4) << bit;
		}
	}
}


TEST(DisplayPort, AirspeedSetpointUsesIconUnitsAndDefaultPositionBesideThrottle)
{
	for (bool inav : {false, true}) {
		for (bool imperial : {false, true}) {
			Capture capture;
			DisplayPort display(Capture::write, &capture);
			DisplaySettings settings;
			settings.symbols = (1u << AIRSPEED_SP) | (1u << THROTTLE);
			settings.inav_font = inav;
			settings.imperial = imperial;
			EXPECT_EQ(settings.positions[AIRSPEED_SP].x, 8);
			EXPECT_EQ(settings.positions[AIRSPEED_SP].y, 19);
			OsdData data = sample();
			data.throttle_percent = 100.f;
			ASSERT_TRUE(display.render(data, settings));
			EXPECT_TRUE(capture.valid);
			EXPECT_NE(capture.line(19).substr(1, 6).find("100%"), std::string::npos);
			EXPECT_EQ(capture.screen[19][7], ' ');
			const char unit = inav ? (imperial ? '\x91' : '\x8f') : (imperial ? '\x9d' : '\x9f');
			const std::string prefix = inav ? "\x8c>" : "AS>";
			const std::string expected = prefix + (imperial ? "49.2" : "22.0") + unit;
			EXPECT_EQ(capture.line(19).substr(8, expected.size()), expected);

			for (float invalid : {NAN, -1.f, INFINITY, std::numeric_limits<float>::max()}) {
				data.airspeed_setpoint_m_s = invalid;
				ASSERT_TRUE(display.render(data, settings));
				const std::string unavailable = prefix + "--" + unit;
				EXPECT_EQ(capture.line(19).substr(8, unavailable.size()), unavailable);
			}

			data.airspeed_setpoint_m_s = 22.f;
			settings.positions[AIRSPEED_SP] = {3, 12};
			ASSERT_TRUE(display.render(data, settings));
			EXPECT_EQ(capture.line(19).substr(8, 10), std::string(10, ' '));
			EXPECT_EQ(capture.line(12).substr(3, expected.size()), expected);
		}
	}
}
