// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team.
#pragma once

#include "OsdData.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/fixed_wing_airspeed_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

namespace msp_osd
{

/** Deterministic state/validity conversion, separate from uORB for testing. */
class OsdTelemetryCore
{
public:
	struct Settings {
		int log_level{6};
		uint32_t scroll_ms{125};
		uint32_t dwell_ms{500};
		uint32_t message_duration_ms{10000};
	};

	struct Samples {
		battery_status_s battery{};
		airspeed_validated_s airspeed{};
		tecs_status_s tecs{};
		fixed_wing_airspeed_status_s airspeed_status{};
		vehicle_local_position_s position{};
		vehicle_global_position_s global{};
		home_position_s home{};
		vehicle_attitude_s attitude{};
		vehicle_status_s status{};
		vehicle_land_detected_s land{};
		vehicle_thrust_setpoint_s thrust[2] {};
		sensor_gps_s gnss{};
		input_rc_s rc{};
	};

	void update(uint64_t now, const Samples &samples, const Settings &settings);
	void pushMessage(uint64_t now, const mavlink_log_s &message, const Settings &settings);
	const OsdData &data() const { return _data; }

private:
	static constexpr unsigned MESSAGE_SLOTS = 4;
	static constexpr unsigned MESSAGE_LENGTH = sizeof(mavlink_log_s::text);
	static constexpr unsigned DISPLAY_LENGTH = sizeof(OsdData::message) - 1;
	static constexpr uint64_t SECOND_US = 1000000;

	struct Message {
		uint64_t timestamp{0};
		uint64_t expires{0};
		uint32_t id{0};
		uint8_t severity{0};
		uint8_t length{0};
		char text[MESSAGE_LENGTH + 1] {};
	};

	void updateTimer(uint64_t now, const vehicle_status_s &status, const vehicle_land_detected_s &land);
	void updateMessages(uint64_t now, const Settings &settings);
	static bool fresh(uint64_t now, uint64_t timestamp, uint64_t maximum_age);
	static bool coordinatesValid(double lat, double lon);
	static float thrustMagnitude(uint64_t now, const vehicle_thrust_setpoint_s &thrust);

	OsdData _data{};
	Message _messages[MESSAGE_SLOTS] {};
	uint32_t _last_message_id{0};
	uint32_t _shown_message_id{0};
	uint64_t _shown_since{0};
	uint64_t _takeoff_time{0};
	uint32_t _flight_seconds{0};
	bool _flight_running{false};
};

class OsdTelemetry
{
public:
	using Settings = OsdTelemetryCore::Settings;
	void update(uint64_t now, const Settings &settings);
	const OsdData &data() const { return _core.data(); }

private:
	OsdTelemetryCore _core;
	OsdTelemetryCore::Samples _samples;
	uORB::Subscription _battery_sub{ORB_ID(battery_status), 0};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _tecs_sub{ORB_ID(tecs_status)};
	uORB::Subscription _airspeed_status_sub{ORB_ID(fixed_wing_airspeed_status)};
	uORB::Subscription _position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _global_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _home_sub{ORB_ID(home_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _thrust_0_sub{ORB_ID(vehicle_thrust_setpoint), 0};
	uORB::Subscription _thrust_1_sub{ORB_ID(vehicle_thrust_setpoint), 1};
	uORB::Subscription _gnss_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _log_sub{ORB_ID(mavlink_log)};
};

} // namespace msp_osd
