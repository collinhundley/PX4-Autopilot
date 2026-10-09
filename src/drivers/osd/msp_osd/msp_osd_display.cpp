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
		display.camera_pitch_deg = _param_osd_cam_pitch.get();
		display.vertical_fov_deg = _param_osd_cam_vfov.get();

		bool reset_positions = false;
		const auto coordinate = [&reset_positions](auto & parameter, int32_t maximum) {
			// Earlier firmware allowed -1. Migrate saved negative coordinates to
			// this parameter's fixed default, preserving all valid custom positions.
			if (parameter.get() < 0) {
				parameter.reset();
				reset_positions = true;
			}

			return static_cast<int16_t>(math::constrain<int32_t>(parameter.get(), 0, maximum));
		};

		display.positions[msp_osd::DISARMED] = {coordinate(_param_osd_pos_arm_x, 59),
							coordinate(_param_osd_pos_arm_y, 21)
						       };
		display.positions[msp_osd::GPS_LAT] = {coordinate(_param_osd_pos_lat_x, 59),
						       coordinate(_param_osd_pos_lat_y, 21)
						      };
		display.positions[msp_osd::GPS_LON] = {coordinate(_param_osd_pos_lon_x, 59),
						       coordinate(_param_osd_pos_lon_y, 21)
						      };
		display.positions[msp_osd::GPS_SATS] = {coordinate(_param_osd_pos_sats_x, 59),
							coordinate(_param_osd_pos_sats_y, 21)
						       };
		display.positions[msp_osd::GPS_SPEED] = {coordinate(_param_osd_pos_gspd_x, 59),
							 coordinate(_param_osd_pos_gspd_y, 21)
							};
		display.positions[msp_osd::HOME_DIST] = {coordinate(_param_osd_pos_hdist_x, 59),
							 coordinate(_param_osd_pos_hdist_y, 21)
							};
		display.positions[msp_osd::HOME_DIR] = {coordinate(_param_osd_pos_hdir_x, 59),
							coordinate(_param_osd_pos_hdir_y, 21)
						       };
		display.positions[msp_osd::MAIN_BATT_VOLTAGE] = {coordinate(_param_osd_pos_volt_x, 59),
								 coordinate(_param_osd_pos_volt_y, 21)
								};
		display.positions[msp_osd::CURRENT_DRAW] = {coordinate(_param_osd_pos_curr_x, 59),
							    coordinate(_param_osd_pos_curr_y, 21)
							   };
		display.positions[msp_osd::MAH_DRAWN] = {coordinate(_param_osd_pos_mah_x, 59),
							 coordinate(_param_osd_pos_mah_y, 21)
							};
		display.positions[msp_osd::RSSI_VALUE] = {coordinate(_param_osd_pos_rssi_x, 59),
							  coordinate(_param_osd_pos_rssi_y, 21)
							 };
		display.positions[msp_osd::ALTITUDE] = {coordinate(_param_osd_pos_alt_x, 59),
							coordinate(_param_osd_pos_alt_y, 21)
						       };
		display.positions[msp_osd::NUMERICAL_VARIO] = {coordinate(_param_osd_pos_vspd_x, 59),
							       coordinate(_param_osd_pos_vspd_y, 21)
							      };
		display.positions[msp_osd::FLYMODE] = {coordinate(_param_osd_pos_mode_x, 59),
						       coordinate(_param_osd_pos_mode_y, 21)
						      };
		display.positions[msp_osd::PITCH_ANGLE] = {coordinate(_param_osd_pos_pitch_x, 59),
							   coordinate(_param_osd_pos_pitch_y, 21)
							  };
		display.positions[msp_osd::ROLL_ANGLE] = {coordinate(_param_osd_pos_roll_x, 59),
							  coordinate(_param_osd_pos_roll_y, 21)
							 };
		display.positions[msp_osd::CROSSHAIRS] = {coordinate(_param_osd_pos_cross_x, 59),
							  coordinate(_param_osd_pos_cross_y, 21)
							 };
		display.positions[msp_osd::AVG_CELL_VOLTAGE] = {coordinate(_param_osd_pos_cell_x, 59),
								coordinate(_param_osd_pos_cell_y, 21)
							       };
		display.positions[msp_osd::HORIZON_SIDEBARS] = {coordinate(_param_osd_pos_sbar_x, 59),
								coordinate(_param_osd_pos_sbar_y, 21)
							       };
		display.positions[msp_osd::POWER] = {coordinate(_param_osd_pos_power_x, 59),
						     coordinate(_param_osd_pos_power_y, 21)
						    };
		display.positions[msp_osd::FLIGHT_TIME] = {coordinate(_param_osd_pos_time_x, 59),
							   coordinate(_param_osd_pos_time_y, 21)
							  };
		display.positions[msp_osd::AIRSPEED] = {coordinate(_param_osd_pos_aspd_x, 59),
							coordinate(_param_osd_pos_aspd_y, 21)
						       };
		display.positions[msp_osd::AIRSPEED_SP] = {coordinate(_param_osd_pos_asp_sp_x, 59),
							   coordinate(_param_osd_pos_asp_sp_y, 21)
							  };
		display.positions[msp_osd::ARTIFICIAL_HORIZON] = {coordinate(_param_osd_pos_horiz_x, 59),
								  coordinate(_param_osd_pos_horiz_y, 21)
								 };
		display.positions[msp_osd::MESSAGES] = {coordinate(_param_osd_pos_msg_x, 59),
							coordinate(_param_osd_pos_msg_y, 21)
						       };
		display.positions[msp_osd::THROTTLE] = {coordinate(_param_osd_pos_thr_x, 59),
							coordinate(_param_osd_pos_thr_y, 21)
						       };
		display.positions[msp_osd::BATT_COMP_VOLTAGE] = {coordinate(_param_osd_pos_cvolt_x, 59),
								 coordinate(_param_osd_pos_cvolt_y, 21)
								};
		display.positions[msp_osd::BATT_CELL_COMP_VOLTAGE] = {coordinate(_param_osd_pos_ccell_x, 59),
								      coordinate(_param_osd_pos_ccell_y, 21)
								     };
		display.positions[msp_osd::BATT_PERC] = {coordinate(_param_osd_pos_batpct_x, 59),
							 coordinate(_param_osd_pos_batpct_y, 21)
							};

		if (reset_positions) { param_notify_changes(); }

		if (_param_osd_canvas.get() == 1) {
			display.columns = 30;
			display.rows = 16;

		} else if (_param_osd_canvas.get() == 2) {
			display.columns = 50;
			display.rows = 18;

		} else if (_param_osd_canvas.get() == 3) {
			display.columns = 60;
			display.rows = 22;

		} else if (_param_osd_canvas.get() == 0 && _canvas_columns != 0) {
			display.columns = _canvas_columns;
			display.rows = _canvas_rows;
		}

		// Profile 4 and Auto without an announcement use the DJI 53x20 default.

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
