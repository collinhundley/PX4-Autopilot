// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.

#include "msp_osd.hpp"
#include <mathlib/mathlib.h>
#include <cmath>

bool MspOsd::display_write(void *context, const uint8_t *payload, size_t size)
{
	auto *self = static_cast<MspOsd *>(context);
	const bool accepted = self->_msp.Send(MSP_CMD_DISPLAYPORT, payload, size);

	if (accepted) {
		++self->_performance_data.successful_sends;

	} else {
		++self->_performance_data.unsuccessful_sends;
	}

	return accepted;
}

void MspOsd::SendDisplay(uint64_t now)
{
	msp_osd::OsdTelemetry::Settings settings{};
	settings.log_level = _param_osd_log_level.get();
	settings.scroll_ms = math::constrain<int32_t>(_param_osd_scroll_rate.get(), 100, 1000);
	settings.dwell_ms = math::constrain<int32_t>(_param_osd_dwell_time.get(), 100, 10000);
	settings.message_duration_ms = 1000u * math::constrain<int32_t>(_param_osd_msg_time.get(), 1, 60);
	_telemetry.update(now, settings);
	const int flush_result = _msp.Flush();

	if (flush_result < 0 && flush_result != -EAGAIN) {
		++_performance_data.unsuccessful_sends;
	}

	// Finish queued packets before constructing another frame. Keep latency
	// bounded instead of accumulating obsolete frames behind a congested UART.
	if (_msp.pending_bytes() != 0) {
		++_congested_frames;
		return;
	}

	SendTelemetry();

	if (_param_osd_symbols.get() == 0) {
		if (_display_active) {
			if (_renderer.release()) {
				_display_active = false;
			}
		}

	} else {
		msp_osd::DisplaySettings display{};
		display.symbols = static_cast<uint32_t>(_param_osd_symbols.get());
		display.imperial = _param_osd_units.get() == 1;
		display.inav_font = _param_osd_font.get() == 1;
		display.crosshair_offset = _param_osd_ch_height.get();
		display.camera_pitch_deg = _param_osd_cam_pitch.get();
		display.vertical_fov_deg = _param_osd_cam_vfov.get();

		if (_param_osd_canvas.get() == 1) {
			display.columns = 30;
			display.rows = 16;

		} else if (_param_osd_canvas.get() == 2) {
			display.columns = 50;
			display.rows = 18;

		} else if (_param_osd_canvas.get() == 0 && _canvas_columns != 0) {
			display.columns = _canvas_columns;
			display.rows = _canvas_rows;
		}

		if (_renderer.render(_telemetry.data(), display)) {
			_display_active = true;
		}
	}

	const int result = _msp.Flush();

	if (result < 0 && result != -EAGAIN) {
		++_performance_data.unsuccessful_sends;
	}
}

void MspOsd::SendTelemetry()
{
	const auto &data = _telemetry.data();
	const uint64_t now = hrt_absolute_time();
	// Digital displays may select their font family from the compatibility identifier.
	const char *variant = _param_osd_font.get() == 1 ? "INAV" : "BTFL";
	Send(MSP_FC_VARIANT, variant, 4);

	// Keep DJI arming/battery telemetry independent of overlay item selection.
	if (data.status_valid) {
		msp_status_t status{};

		if (data.armed) {
			status.flightModeFlags |= (1 << MSP_MODE_ARM);
		}

		Send(MSP_STATUS, &status, sizeof(status));
	}

	battery_status_s battery{};
	_battery_status_sub.copy(&battery);
	msp_battery_state_t message{};
	message.batteryState = 3; // MSP BATTERY_NOT_PRESENT until a fresh battery is connected.

	if (battery.connected && battery.timestamp != 0 && now >= battery.timestamp && now - battery.timestamp <= 3_s) {
		auto bounded = [](float value, float scale, float maximum) -> uint16_t {
			return std::isfinite(value) && value >= 0.f ? static_cast<uint16_t>(fminf(value * scale, maximum)) : 0;
		};
		message.batteryCellCount = battery.cell_count > 0 ? battery.cell_count : 0;
		message.batteryCapacity = bounded(battery.capacity, 1.f, 65535.f);
		message.mAhDrawn = bounded(battery.discharged_mah, 1.f, 65535.f);
		message.amperage = bounded(battery.current_a, 100.f, 32767.f);
		message.legacyBatteryVoltage = bounded(battery.voltage_v, 10.f, 255.f);
		message.batteryVoltage = bounded(battery.voltage_v, 100.f, 65535.f);
		const bool critical = (battery.warning >= battery_status_s::WARNING_CRITICAL &&
				       battery.warning <= battery_status_s::WARNING_FAILED) || battery.warning == battery_status_s::STATE_UNHEALTHY;
		message.batteryState = critical ? 2 : battery.warning == battery_status_s::WARNING_LOW ? 1 : 0;
	}

	Send(MSP_BATTERY_STATE, &message, sizeof(message));

	if (_param_osd_rc_stick.get() == 1 && data.status_valid && !data.armed) {
		input_rc_s rc{};
		_input_rc_sub.copy(&rc);

		if (!rc.rc_lost && !rc.rc_failsafe && rc.channel_count >= 4 && rc.timestamp_last_signal != 0 &&
		    now >= rc.timestamp_last_signal && now - rc.timestamp_last_signal <= 500_ms) {
			msp_rc_t channels{};
			channels.channelValue[0] = rc.values[0];
			channels.channelValue[1] = rc.values[1];
			channels.channelValue[2] = rc.values[3];
			channels.channelValue[3] = rc.values[2];
			Send(MSP_RC, &channels, sizeof(channels));
		}
	}
}
