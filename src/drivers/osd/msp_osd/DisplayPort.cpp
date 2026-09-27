// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#include "DisplayPort.hpp"

#include <cstdio>
#include <cstring>

namespace msp_osd
{
namespace
{
constexpr float PI = 3.14159265358979323846f;
constexpr float DEGREES = 180.f / PI;
constexpr float FEET_PER_METER = 3.280839895f;
constexpr float MPH_PER_M_S = 2.236936292f;
constexpr int MAX_TEXT = 30;
constexpr int HORIZON_HALF_WIDTH = 5;
// Betaflight page-0 glyph assignments: https://betaflight.com/docs/development/OSD-Glyps
constexpr uint8_t ARROW_SOUTH = 0x60;
constexpr uint8_t HORIZON_BAR = 0x80; // Nine horizontal strokes, from top to bottom within a cell.
constexpr uint8_t SIDEBAR = 0x13;
constexpr uint8_t RETICLE[] = {0x72, 0x73, 0x74};
constexpr uint8_t HEARTBEAT = 0;
constexpr uint8_t RELEASE = 1;
constexpr uint8_t CLEAR = 2;
constexpr uint8_t WRITE_STRING = 3;
constexpr uint8_t DRAW = 4;
constexpr uint8_t OPTIONS = 5;

bool finite_range(float value, float minimum, float maximum)
{
	return std::isfinite(value) && value >= minimum && value <= maximum;
}

void format_number(char *buffer, size_t size, int width, const char *prefix, float value, int decimals,
		   const char *unit, float minimum, float maximum)
{
	int length = -1;

	if (finite_range(value, minimum, maximum)) {
		length = snprintf(buffer, size, "%s%.*f%s", prefix, decimals, static_cast<double>(value), unit);
	}

	if (length < 0 || length > width || length >= static_cast<int>(size)) {
		snprintf(buffer, size, "%s--%s", prefix, unit);
	}
}
}

bool DisplayPort::command(uint8_t subcommand)
{
	return _write && _write(_context, &subcommand, 1);
}

bool DisplayPort::clear()
{
	return command(CLEAR) && command(DRAW);
}

bool DisplayPort::release()
{
	return clear() && command(RELEASE);
}

uint16_t DisplayPort::glyph(uint8_t character) const
{
	// Internal strings use Betaflight page 0, plus private altitude and airspeed tokens.
	// Wire glyph IDs follow the public font maps, not Unicode:
	// github.com/iNavFlight/inav/blob/master/src/main/drivers/osd_symbols.h
	if (!_inav_font) {
		return character == 0xe0 ? 0x0c : character == 0xe1 ? 0x0f : character;
	}

	if (character >= ARROW_SOUTH && character < ARROW_SOUTH + 16) {
		// INAV starts at north and runs clockwise; Betaflight starts south and runs counterclockwise.
		return 0x13c + ((8 - (character - ARROW_SOUTH) + 16) % 16);
	}

	if (character >= HORIZON_BAR && character <= HORIZON_BAR + 8) {
		return 0x14c + character - HORIZON_BAR;
	}

	switch (character) {
	case 0x04: return 0x95; // throttle

	case 0x06: return 0x1f; // volts

	case 0x07: return 0x99; // mAh

	case 0x0c: return 0x82; // meters

	case 0x0f: return 0x74; // feet

	case 0x11: return 0x10; // home

	case SIDEBAR: return 0x131;

	case 0x70: return 0x17; // ground-speed direction marker

	case 0x72: return 0x13a; // reticle left

	case 0x73: return 0x166; // reticle center

	case 0x74: return 0x13b; // reticle right

	case 0x99: return 0x8d; // feet/second

	case 0x9a: return 0x6a; // amps

	case 0x9c: return 0x9f; // flight minutes

	case 0x9d: return 0x91; // mph

	case 0x9f: return 0x8f; // meters/second

	case 0xe0: return 0x76; // altitude meters

	case 0xe1: return 0x78; // altitude feet

	case 0xe2: return 0x8c; // airspeed icon (INAV only)

	case 0xf3: return 0xd3; // Ah (INAV only)

	// These ASCII punctuation slots contain unit symbols in the INAV font.
	case '"':
	case '$':
	case '\'':
	case '?': return '-';

	default: return character;
	}
}

bool DisplayPort::write(int x, int y, const uint8_t *value, size_t length)
{
	if (y < 0 || y >= _rows || x >= _columns) {
		return true;
	}

	if (x < 0) {
		const size_t skip = static_cast<size_t>(-x);

		if (skip >= length) {
			return true;
		}

		value += skip;
		length -= skip;
		x = 0;
	}

	if (length > static_cast<size_t>(_columns - x)) {
		length = _columns - x;
	}

	while (length > 0) {
		const uint8_t page = glyph(value[0]) >> 8;
		// Protocol strings are NUL terminated, with at most 30 visible characters.
		uint8_t payload[4 + MAX_TEXT + 1] = {WRITE_STRING, static_cast<uint8_t>(y), static_cast<uint8_t>(x), page};
		size_t count = 0;

		// A DisplayPort string has one font page. Split only at page changes or the packet limit.
		while (count < length && count < MAX_TEXT && (glyph(value[count]) >> 8) == page) {
			payload[4 + count] = static_cast<uint8_t>(glyph(value[count]));
			++count;
		}

		if (!_write || !_write(_context, payload, count + 5)) {
			return false;
		}

		x += count;
		value += count;
		length -= count;
	}

	return true;
}

bool DisplayPort::text(Position position, const char *value, size_t capacity)
{
	uint8_t clean[MAX_TEXT];
	size_t length = 0;

	while (length < capacity && length < sizeof(clean) && length < static_cast<size_t>(position.width)
	       && value[length] != '\0') {
		uint8_t character = value[length];

		if (character >= 'a' && character <= 'z') {
			character -= 'a' - 'A';
		}

		// Lowercase ASCII overlaps the font's arrows and horizon glyphs.
		clean[length++] = (character >= ' ' && character <= '_') ? character : '?';
	}

	return write(position.x, position.y, clean, length);
}

bool DisplayPort::formatted(Position position, const char *value, bool right_align)
{
	// Only trusted renderer-generated strings may contain glyph bytes. User text goes through text().
	const size_t length = strnlen(value, MAX_TEXT);

	if (length > static_cast<size_t>(position.width)) {
		return false;
	}

	const int x = right_align ? position.x + position.width - static_cast<int>(length) : position.x;
	return write(x, position.y, reinterpret_cast<const uint8_t *>(value), length);
}

bool DisplayPort::number(Position position, const char *prefix, float value, int decimals, const char *unit,
			 float minimum, float maximum, bool right_align)
{
	char buffer[MAX_TEXT + 1];
	format_number(buffer, sizeof(buffer), position.width, prefix, value, decimals, unit, minimum, maximum);
	return formatted(position, buffer, right_align);
}

bool DisplayPort::camera_vectors(const OsdData &data, const DisplaySettings &settings, float down[3], float forward[2])
{
	float norm_squared = 0.f;

	for (float value : data.attitude_q) {
		if (!std::isfinite(value)) {
			return false;
		}

		norm_squared += value * value;
	}

	if (!finite_range(norm_squared, 0.25f, 4.f)) {
		return false;
	}

	const float inverse_norm = 1.f / sqrtf(norm_squared);
	const float w = data.attitude_q[0] * inverse_norm;
	const float x = data.attitude_q[1] * inverse_norm;
	const float y = data.attitude_q[2] * inverse_norm;
	const float z = data.attitude_q[3] * inverse_norm;
	const float pitch = finite_range(settings.camera_pitch_deg, -180.f, 180.f) ? settings.camera_pitch_deg / DEGREES : 0.f;
	const float c = cosf(pitch);
	const float s = sinf(pitch);
	// Body-to-NED quaternion. Camera axes are X forward, Y right, Z down;
	// a positive fixed mounting angle points the camera above body X.
	const float body_down_x = 2.f * (x * z - w * y);
	const float body_down_z = 1.f - 2.f * (x * x + y * y);
	down[0] = c * body_down_x - s * body_down_z;
	down[1] = 2.f * (y * z + w * x);
	down[2] = s * body_down_x + c * body_down_z;
	forward[0] = c * (1.f - 2.f * (y * y + z * z)) - s * 2.f * (x * z + w * y);
	forward[1] = c * 2.f * (x * y + w * z) - s * 2.f * (y * z - w * x);
	return true;
}

bool DisplayPort::horizon(const OsdData &data, const DisplaySettings &settings, int center_x, int center_y, int half_height)
{
	float down[3];
	float forward[2];

	if (!camera_vectors(data, settings, down, forward)) {
		return text({center_x - 5, center_y + half_height, 11}, "HORIZON --");
	}

	const float fov = finite_range(settings.vertical_fov_deg, 10.f, 170.f) ? settings.vertical_fov_deg : 60.f;
	const float vertical_cell = 2.f * tanf(fov / (2.f * DEGREES)) / _rows;
	// The video is 16:9; derive the cell aspect ratio from the negotiated canvas.
	const float horizontal_cell = vertical_cell * (16.f / 9.f) * _rows / _columns;
	const float a = down[1] * horizontal_cell;
	const float b = down[2] * vertical_cell;

	// A camera ray (1, u, v) is on the horizon when dot(down, ray) == 0.
	// Select the stable line parameterization, including rolls near +/-90 degrees.
	if (fabsf(b) >= fabsf(a) && fabsf(b) > 1e-6f) {
		uint8_t run[2 * HORIZON_HALF_WIDTH + 1];
		int run_start = 0;
		int run_row = 0;
		size_t run_length = 0;

		for (int column = -HORIZON_HALF_WIDTH; column <= HORIZON_HALF_WIDTH; ++column) {
			const float line = -(down[0] + a * column) / b;

			if (line < -half_height - 0.5f || line >= half_height + 0.5f) {
				continue;
			}

			const int row = static_cast<int>(floorf(line + 0.5f));
			const int subcell = static_cast<int>((line - row + 0.5f) * 9.f);
			const uint8_t glyph = HORIZON_BAR + (subcell < 0 ? 0 : subcell > 8 ? 8 : subcell);

			if (run_length > 0 && row != run_row) {
				if (!write(center_x + run_start, center_y + run_row, run, run_length)) {
					return false;
				}

				run_length = 0;
			}

			if (run_length == 0) {
				run_start = column;
				run_row = row;
			}

			run[run_length++] = glyph;
		}

		if (run_length > 0 && !write(center_x + run_start, center_y + run_row, run, run_length)) {
			return false;
		}

	} else if (fabsf(a) > 1e-6f) {
		for (int row = -half_height; row <= half_height; ++row) {
			const float line = -(down[0] + b * row) / a;

			if (line < -HORIZON_HALF_WIDTH - 0.5f || line >= HORIZON_HALF_WIDTH + 0.5f) {
				continue;
			}

			const int column = static_cast<int>(floorf(line + 0.5f));

			if (!write(center_x + column, center_y + row, &SIDEBAR, 1)) {
				return false;
			}
		}
	}

	return true;
}

bool DisplayPort::render(const OsdData &data, const DisplaySettings &settings)
{
	// Reject corrupt negotiation rather than deriving negative coordinates or huge packets.
	_columns = settings.columns >= 30 && settings.columns <= 60 ? settings.columns : 60;
	_rows = settings.rows >= 16 && settings.rows <= 22 ? settings.rows : 22;
	_inav_font = settings.inav_font;

	if (settings.symbols == 0) {
		return release();
	}

	if (!command(HEARTBEAT)) {
		return false;
	}

	// Tell compatible displays which HD grid to use, rather than placing HD coordinates on an SD grid.
	const bool standard_canvas = (_columns == 60 && _rows == 22) || (_columns == 50 && _rows == 18)
				     || (_columns == 30 && _rows == 16);

	if (standard_canvas) {
		const uint8_t options[] {OPTIONS, 0, static_cast<uint8_t>(_columns == 60 ? 3 : _columns == 50 ? 1 : 0)};

		if (!_write || !_write(_context, options, sizeof(options))) { return false; }
	}

	if (!command(CLEAR)) { return false; }

	const bool large = _columns >= 50 && _rows >= 18;
	const int center_x = _columns / 2;
	const int center_y = large ? (_rows * 8) / 22 : 6;
	const int half_height = large ? 3 : 2;
	const int instruments_y = center_y - 1;
	const int ground_speed_y = instruments_y + 2; // One blank row between airspeed and ground speed.
	const int vario_y = large ? (_rows * 13) / 22 : 10;
	const int bottom_y = _rows - 4;
	const int home_x = large ? center_x - 4 : 0;
	const int home_y = large ? 0 : 1;
	const float distance_scale = settings.imperial ? FEET_PER_METER : 1.f;
	const float speed_scale = settings.imperial ? MPH_PER_M_S : 1.f;
	const char *distance_unit = settings.imperial ? "\x0f" : "\x0c";
	const char *altitude_unit = settings.imperial ? "\xe1" : "\xe0";
	const char *speed_unit = settings.imperial ? "\x9d" : "\x9f";
	const char *vario_unit = settings.imperial ? "\x99" : "\x9f";
	const auto enabled = [&settings](SymbolIndex symbol) { return (settings.symbols & (1u << symbol)) != 0; };
	const auto position = [large](Position hd, Position sd) { return large ? hd : sd; };

	// The former PX4-label bit stays reserved. Mode and state now share the top-left line.
	const int left = large ? 1 : 0;
	const int header_width = large ? center_x - 5 : _columns - 12;
	const char *state = !data.status_valid ? "--" : data.armed ? (data.failsafe ? "ARM!" : "ARM") :
			    (data.failsafe ? "DIS!" : "DIS");
	const int state_length = enabled(DISARMED) ? strlen(state) : 0;
	int state_x = left;

	if (enabled(FLYMODE)) {
		const char *mode = data.mode[0] ? data.mode : "MODE --";
		const int mode_width = header_width - (state_length > 0 ? state_length + 1 : 0);
		const int mode_length = strnlen(mode, sizeof(data.mode));

		if (!text({left, 0, mode_width}, mode, sizeof(data.mode))) { return false; }

		state_x += (mode_length < mode_width ? mode_length : mode_width) + 1;
	}

	if (state_length > 0 && !text({state_x, 0, state_length}, state)) { return false; }

	if (enabled(FLIGHT_TIME)) {
		char buffer[MAX_TEXT + 1];

		if (!data.flight_time_valid) {
			snprintf(buffer, sizeof(buffer), "\x9c --:--");

		} else if (data.flight_seconds < 3600) {
			snprintf(buffer, sizeof(buffer), "\x9c %02u:%02u", static_cast<unsigned>(data.flight_seconds / 60),
				 static_cast<unsigned>(data.flight_seconds % 60));

		} else if (data.flight_seconds < 100 * 3600) {
			snprintf(buffer, sizeof(buffer), "\x9c %02u:%02u:%02u", static_cast<unsigned>(data.flight_seconds / 3600),
				 static_cast<unsigned>((data.flight_seconds / 60) % 60), static_cast<unsigned>(data.flight_seconds % 60));

		} else {
			snprintf(buffer, sizeof(buffer), "\x9c >99H");
		}

		if (!formatted({_columns - 11, 0, 10}, buffer, true)) { return false; }
	}

	if (enabled(HOME_DIR)) {
		float down[3];
		float forward[2];
		uint8_t arrow = '?';

		if (std::isfinite(data.home_bearing_rad) && camera_vectors(data, settings, down, forward)
		    && forward[0] * forward[0] + forward[1] * forward[1] > 0.0025f) {
			const float relative = remainderf(data.home_bearing_rad - atan2f(forward[1], forward[0]), 2.f * PI);
			const int sector = static_cast<int>(roundf(relative * (8.f / PI)));
			arrow = ARROW_SOUTH + ((8 - sector + 16) % 16);
		}

		if (!write(home_x + 1, home_y, &arrow, 1)) { return false; }
	}

	if (enabled(HOME_DIST)) {
		const uint8_t home = 0x11;

		if (!write(home_x, home_y, &home, 1)
		    || !number({home_x + 3, home_y, large ? 12 : 16}, "", data.home_distance_m * distance_scale,
			       0, distance_unit, 0.f, 1e9f)) { return false; }
	}

	if (enabled(MESSAGES) && !text({left, 2, _columns - 2 * left}, data.message, sizeof(data.message))) { return false; }

	if (enabled(ARTIFICIAL_HORIZON) && !horizon(data, settings, center_x, center_y, half_height)) { return false; }

	if (enabled(HORIZON_SIDEBARS)) {
		for (int row = -half_height; row <= half_height; ++row) {
			if (!write(center_x - HORIZON_HALF_WIDTH - 2, center_y + row, &SIDEBAR, 1)
			    || !write(center_x + HORIZON_HALF_WIDTH + 2, center_y + row, &SIDEBAR, 1)) { return false; }
		}
	}

	if (enabled(CROSSHAIRS)) {
		const int maximum_offset = _rows - center_y - 1;
		const int offset = settings.crosshair_offset < -center_y ? -center_y :
				   settings.crosshair_offset > maximum_offset ? maximum_offset : settings.crosshair_offset;

		if (!write(center_x - 1, center_y + offset, RETICLE, sizeof(RETICLE))) { return false; }
	}

	const Position speed_position {large ? center_x - 17 : 0, instruments_y, large ? 10 : 8};

	if (enabled(GPS_SPEED) && !number({speed_position.x, ground_speed_y, speed_position.width}, "\x70 ",
					  data.ground_speed_m_s * speed_scale, 1, speed_unit, 0.f, 9999.f, true)) { return false; }

	const char *airspeed_prefix = _inav_font ? (data.airspeed_estimated ? "\xe2 *" : "\xe2 ") :
				      (data.airspeed_estimated ? "AS*" : "AS ");

	if (enabled(AIRSPEED) && !number(speed_position, airspeed_prefix,
					 data.airspeed_m_s * speed_scale, 1, speed_unit, 0.f, 9999.f, true)) { return false; }

	if (enabled(ALTITUDE) && !number({center_x + 8, instruments_y, _columns - center_x - 8}, "",
					 data.altitude_m * distance_scale, 0, altitude_unit, -1e7f, 1e7f)) { return false; }

	if (enabled(NUMERICAL_VARIO) && !number({_columns - 12, vario_y, 11}, "VS ",
						data.vertical_speed_m_s * distance_scale, 1, vario_unit, -9999.f, 9999.f, true)) { return false; }

	if (enabled(THROTTLE) && !number({left, _rows - 1, 9}, "\x04 ", data.throttle_percent,
					 0, "%", 0.f, 100.f)) { return false; }

	const int capacity_x = large ? center_x - 9 : 11;
	const int current_x = large ? center_x + 3 : 22;
	const int current_width = large ? 9 : 8;

	// Center the voltage pair beneath the combined capacity/current field area.
	char pack_voltage[8] {};
	char cell_voltage[8] {};
	char voltage_pair[MAX_TEXT + 1];
	const bool pack_enabled = enabled(MAIN_BATT_VOLTAGE);
	const bool cell_enabled = enabled(AVG_CELL_VOLTAGE);

	if (pack_enabled) {
		format_number(pack_voltage, sizeof(pack_voltage), 7, "", data.battery_voltage, 1, "V", 0.f, 999.f);
	}

	if (cell_enabled) {
		format_number(cell_voltage, sizeof(cell_voltage), 7, "", data.cell_voltage, 2, "V", 0.f, 9.99f);
	}

	snprintf(voltage_pair, sizeof(voltage_pair), "%s%s%s", pack_voltage,
		 pack_enabled && cell_enabled ? "/" : "", cell_voltage);
	const int voltage_length = strlen(voltage_pair);
	const int battery_width = current_x + current_width - capacity_x;
	const int voltage_x = capacity_x + (battery_width - voltage_length) / 2;

	if (enabled(MAH_DRAWN)) {
		const bool amp_hours = finite_range(data.discharged_mah, 10000.f, 1e8f);
		const char *unit = amp_hours ? (_inav_font ? "\xf3" : "AH") : "\x07";

		if (!number({capacity_x, bottom_y, 9}, "", data.discharged_mah / (amp_hours ? 1000.f : 1.f),
			    amp_hours ? 2 : 0, unit, 0.f, 1e8f, true)) { return false; }
	}

	if (voltage_length > 0 && !formatted({voltage_x, bottom_y + 1, voltage_length}, voltage_pair)) { return false; }

	if (enabled(CURRENT_DRAW) && !number({current_x, bottom_y, current_width}, "", data.current_a,
					     1, "\x9a", 0.f, 9999.f)) { return false; }

	if (enabled(PITCH_ANGLE) && !number({left, large ? bottom_y - 2 : 11, 13}, "P ", data.pitch_rad * DEGREES,
					    0, "DEG", -180.f, 180.f)) { return false; }

	if (enabled(ROLL_ANGLE) && !number({_columns - 14, large ? bottom_y - 2 : 11, 13}, "R ", data.roll_rad * DEGREES,
					   0, "DEG", -180.f, 180.f, true)) { return false; }

	const float power = finite_range(data.current_a, 0.f, 9999.f) && finite_range(data.battery_voltage, 0.f, 999.f)
			    ? data.current_a * data.battery_voltage : NAN;

	if (enabled(POWER) && !number(position({30, _rows - 2, 10}, {0, 13, 10}), "", power,
				      0, "W", 0.f, 1e7f)) { return false; }

	if (enabled(RSSI_VALUE) && !number({_columns - 12, large ? _rows - 2 : 15, 11}, "RC ", data.rssi_percent,
					   0, "%", 0.f, 100.f, true)) { return false; }

	if (enabled(GPS_SATS) && !number({_columns - 9, large ? _rows - 3 : 1, 8}, "SAT ", data.satellites,
					 0, "", 0.f, 255.f, true)) { return false; }

	// Retain double precision until formatting GNSS coordinates.
	for (int axis = 0; axis < 2; ++axis) {
		if (enabled(axis == 0 ? GPS_LAT : GPS_LON)) {
			const double value = axis == 0 ? data.latitude_deg : data.longitude_deg;
			const double limit = axis == 0 ? 90.0 : 180.0;
			const char *prefix = axis == 0 ? "LAT " : "LON ";
			char buffer[MAX_TEXT + 1];

			if (std::isfinite(value) && fabs(value) <= limit) {
				snprintf(buffer, sizeof(buffer), "%s%.5f", prefix, value);

			} else {
				snprintf(buffer, sizeof(buffer), "%s--", prefix);
			}

			const int width = large ? _columns / 2 - 2 : 15;
			const Position location {axis == 0 ? left : _columns - width - left, large ? bottom_y - 1 : 14, width};

			if (!formatted(location, buffer, axis == 1)) { return false; }
		}
	}

	return command(DRAW);
}

} // namespace msp_osd
