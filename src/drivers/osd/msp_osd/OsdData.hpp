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
	ARTIFICIAL_HORIZON = 24, MESSAGES = 25, THROTTLE = 26
};

// SI units internally. NaN means unavailable; renderer must never print NaN/Inf.
struct OsdData {
	float battery_voltage{NAN};
	float cell_voltage{NAN};
	float current_a{NAN};
	float discharged_mah{NAN};
	float ground_speed_m_s{NAN};
	float airspeed_m_s{NAN};
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

struct DisplaySettings {
	uint32_t symbols{0};
	bool imperial{true};
	bool inav_font{false}; // Goggles N3 supports the INAV glyph map, including font page 1.
	uint8_t columns{60};
	uint8_t rows{22};
	int crosshair_offset{0};
	float camera_pitch_deg{0.f}; // Positive camera uptilt relative to body X.
	float vertical_fov_deg{60.f};
};

} // namespace msp_osd
