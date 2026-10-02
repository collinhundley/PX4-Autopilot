// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#pragma once

#include <cmath>
#include <cstdint>

namespace msp_osd
{

enum SymbolIndex : uint8_t {
	CRAFT_NAME = 0, DISARMED = 1, GPS_LAT = 2, GPS_LON = 3, GPS_SATS = 4,
	GPS_SPEED = 5, HOME_DIST = 6, HOME_DIR = 7, MAIN_BATT_VOLTAGE = 8,
	CURRENT_DRAW = 9, MAH_DRAWN = 10, RSSI_VALUE = 11, ALTITUDE = 12,
	NUMERICAL_VARIO = 13, FLYMODE = 14, ESC_TMP = 15, PITCH_ANGLE = 16,
	ROLL_ANGLE = 17, CROSSHAIRS = 18, AVG_CELL_VOLTAGE = 19,
	HORIZON_SIDEBARS = 20, POWER = 21, FLIGHT_TIME = 22, AIRSPEED = 23,
	ARTIFICIAL_HORIZON = 24, MESSAGES = 25, THROTTLE = 26,
	BATT_COMP_VOLTAGE = 27, BATT_CELL_COMP_VOLTAGE = 28, BATT_PERC = 29
};

// SI units internally. NaN means unavailable; renderer must never print NaN/Inf.
struct OsdData {
	float battery_voltage{NAN};
	float cell_voltage{NAN};
	float compensated_battery_voltage{NAN};
	float compensated_cell_voltage{NAN};
	float battery_remaining_percent{NAN};
	float current_a{NAN};
	float discharged_mah{NAN};
	float ground_speed_m_s{NAN};
	float airspeed_m_s{NAN}; // Validated calibrated airspeed (CAS).
	bool airspeed_estimated{false};
	float altitude_m{NAN};
	float vertical_speed_m_s{NAN};
	float home_distance_m{NAN};
	float home_bearing_rad{NAN};
	float attitude_q[4] {NAN, NAN, NAN, NAN};
	float roll_rad{NAN};
	float pitch_rad{NAN};
	float throttle_percent{NAN};
	float rssi_percent{NAN};
	double latitude_deg{static_cast<double>(NAN)};
	double longitude_deg{static_cast<double>(NAN)};
	int satellites{-1};
	bool status_valid{false};
	bool armed{false};
	bool failsafe{false};
	uint32_t flight_seconds{0};
	bool flight_time_valid{false};
	char mode[24] {};
	char message[31] {};
};

struct ItemPosition {
	int16_t x{0}; // Zero-based column; graphical items use their centre.
	int16_t y{0}; // Zero-based row; graphical items use their centre.
};

struct DisplaySettings {
	uint32_t symbols{0};
	bool imperial{true};
	bool inav_font{false}; // Goggles N3 supports the INAV glyph map, including font page 1.
	uint8_t columns{53};
	uint8_t rows{20};
	float camera_pitch_deg{0.f}; // Positive camera uptilt relative to body X.
	float vertical_fov_deg{60.f};
	// Fixed defaults match the OSD_POS_* parameters and the agreed 53x20 layout.
	ItemPosition positions[BATT_PERC + 1] {
		{0, 0}, // CRAFT_NAME
		{18, 1}, // DISARMED
		{1, 15}, // GPS_LAT
		{28, 15}, // GPS_LON
		{46, 18}, // GPS_SATS
		{11, 9}, // GPS_SPEED
		{22, 1}, // HOME_DIST
		{23, 1}, // HOME_DIR
		{21, 17}, // MAIN_BATT_VOLTAGE
		{32, 18}, // CURRENT_DRAW
		{24, 18}, // MAH_DRAWN
		{46, 19}, // RSSI_VALUE
		{34, 6}, // ALTITUDE
		{41, 11}, // NUMERICAL_VARIO
		{1, 1}, // FLYMODE
		{0, 0}, // ESC_TMP
		{1, 14}, // PITCH_ANGLE
		{39, 14}, // ROLL_ANGLE
		{26, 7}, // CROSSHAIRS
		{27, 17}, // AVG_CELL_VOLTAGE
		{26, 7}, // HORIZON_SIDEBARS
		{1, 17}, // POWER
		{42, 1}, // FLIGHT_TIME
		{11, 6}, // AIRSPEED
		{26, 7}, // ARTIFICIAL_HORIZON
		{1, 3}, // MESSAGES
		{1, 19}, // THROTTLE
		{20, 19}, // BATT_COMP_VOLTAGE
		{28, 19}, // BATT_CELL_COMP_VOLTAGE
		{16, 18}, // BATT_PERC
	};
};

} // namespace msp_osd
