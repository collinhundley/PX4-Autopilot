// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#pragma once

#include "OsdData.hpp"
#include <cstddef>

namespace msp_osd
{

/** Bounded Betaflight/INAV-font DisplayPort renderer. Write receives an MSP 182 payload. */
class DisplayPort
{
public:
	using Write = bool (*)(void *, const uint8_t *, size_t);
	DisplayPort(Write callback, void *context) : _write(callback), _context(context) {}
	bool render(const OsdData &data, const DisplaySettings &settings);
	bool clear();
	bool release();

private:
	struct Position { int x; int y; int width; };
	bool command(uint8_t command);
	bool write(int x, int y, const uint8_t *text, size_t length);
	bool text(Position position, const char *value, size_t capacity = 31);
	bool formatted(Position position, const char *value);
	bool number(Position position, const char *prefix, float value, int decimals, const char *unit,
		    float minimum, float maximum);
	bool speed(Position position, const char *prefix, float value, const char *unit);
	uint16_t glyph(uint8_t character) const;
	bool horizon(const OsdData &data, const DisplaySettings &settings, int center_x, int center_y, int half_height);
	static bool camera_vectors(const OsdData &data, const DisplaySettings &settings, float down[3], float forward[2]);
	Write _write;
	void *_context;
	int _columns{53};
	int _rows{20};
	bool _inav_font{false};
};

} // namespace msp_osd
