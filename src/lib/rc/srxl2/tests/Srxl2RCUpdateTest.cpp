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

#include <gtest/gtest.h>
#include <modules/rc_update/rc_update.h>
#include <modules/manual_control/ManualControl.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <px4_platform_common/posix.h>
#include <stdio.h>
#include <string.h>

#include "Srxl2.hpp"

class Srxl2TestRCUpdate : public rc_update::RCUpdate
{
public:
	void processOnce() { RCUpdate::Run(); }
	void reloadParameters() { RCUpdate::updateParams(); }
};

class Srxl2TestManualControl : public ManualControl
{
public:
	void processOnce(hrt_abstime now) { ManualControl::processInput(now); }
};

// Exercise the production RC calibration and input selection modules. The only
// adapter below is the same ChannelState -> input_rc contract used by the UART
// driver; no receiver, flight task or work queue needs to run in these tests.
class Srxl2RCUpdateTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		setParameter<int32_t>("RC_CHAN_CNT", 7);
		setParameter<int32_t>("RC_MAP_ROLL", 1);
		setParameter<int32_t>("RC_MAP_PITCH", 2);
		setParameter<int32_t>("RC_MAP_THROTTLE", 3);
		setParameter<int32_t>("RC_MAP_YAW", 4);
		setParameter<int32_t>("RC_FAILS_THR", 0);
		setParameter<int32_t>("COM_RC_IN_MODE", 0);
		setParameter<float>("COM_RC_LOSS_T", .5f);

		for (unsigned channel = 1; channel <= 7; ++channel) {
			char name[20];
			snprintf(name, sizeof(name), "RC%u_MIN", channel);
			setParameter<float>(name, 1102.f);
			snprintf(name, sizeof(name), "RC%u_TRIM", channel);
			setParameter<float>(name, 1500.f);
			snprintf(name, sizeof(name), "RC%u_MAX", channel);
			setParameter<float>(name, 1898.f);
			snprintf(name, sizeof(name), "RC%u_REV", channel);
			setParameter<int32_t>(name, 1);
		}

		_rc_update.reloadParameters();
	}

	template<typename T> void setParameter(const char *name, T value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set(handle, &value), PX4_OK) << name;
	}

	static srxl2::Packet channels(uint32_t mask, uint8_t command = 0, uint16_t roll = 0x8000)
	{
		srxl2::Packet packet {};
		packet.data[0] = 0xa6;
		packet.data[1] = 0xcd;
		packet.data[3] = command;
		packet.data[4] = srxl2::DeviceId;
		packet.data[5] = 83;

		for (unsigned i = 0; i < 4; ++i) {
			packet.data[8 + i] = mask >> (8 * i);
		}

		size_t offset = 12;

		for (unsigned channel = 0; channel < 32; ++channel) {
			if (mask & (uint32_t(1) << channel)) {
				const uint16_t value = channel == 0 ? 0x2aa0 : (channel == 1 ? roll : 0x8000);
				packet.data[offset++] = value;
				packet.data[offset++] = value >> 8;
			}
		}

		packet.length = offset + 2;
		packet.data[2] = packet.length;
		const uint16_t crc = srxl2::crc16(packet.data, offset);
		packet.data[offset] = crc >> 8;
		packet.data[offset + 1] = crc;
		packet.last_sequence = packet.length - 1;
		return packet;
	}

	void receive(const srxl2::Packet &packet, hrt_abstime capture_time, hrt_abstime publish_time)
	{
		ASSERT_TRUE(_endpoint.process(packet, capture_time).channel_data);
		const auto state = _endpoint.channel_state(publish_time);
		input_rc_s input {};
		input.timestamp = publish_time;
		input.timestamp_last_signal = state.timestamp_last_signal;
		input.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SRXL2;
		input.channel_count = state.channel_count;
		input.rc_lost = state.rc_lost;
		input.rc_failsafe = state.rc_failsafe;
		input.rssi = state.rssi;
		input.rssi_dbm = state.rssi_dbm;
		input.rc_lost_frame_count = state.lost_frame_count;
		input.rc_total_frame_count = state.total_frame_count;
		memcpy(input.values, state.values, sizeof(input.values));
		_input_rc_pub.publish(input);
		_rc_update.processOnce();
	}

	void establishSignal()
	{
		const auto initial_time = hrt_absolute_time();
		receive(channels(0x7f), initial_time, initial_time);
		// RCUpdate intentionally rejects the first 100 ms of regained RC.
		px4_usleep(110000);
		const auto second_time = hrt_absolute_time();
		receive(channels(0x7f), second_time, second_time);
		px4_usleep(11000);
		const auto publish_time = hrt_absolute_time();
		_last_signal = publish_time - 2000;
		receive(channels(0x7f, 0, 0xd554), _last_signal, publish_time);
		ASSERT_TRUE(_manual_input_sub.update(&_manual_input));
		ASSERT_TRUE(_manual_input.valid);
	}

	srxl2::Endpoint _endpoint {0x12345678};
	Srxl2TestRCUpdate _rc_update;
	uORB::Publication<input_rc_s> _input_rc_pub {ORB_ID(input_rc)};
	uORB::Subscription _rc_channels_sub {ORB_ID(rc_channels)};
	uORB::Subscription _manual_input_sub {ORB_ID(manual_control_input)};
	uORB::Subscription _manual_setpoint_sub {ORB_ID(manual_control_setpoint)};
	manual_control_setpoint_s _manual_input {};
	hrt_abstime _last_signal {0};
};

TEST_F(Srxl2RCUpdateTest, CalibrationMappingAndCaptureTimeReachManualControl)
{
	establishSignal();
	ASSERT_FALSE(HasFatalFailure());
	EXPECT_EQ(_manual_input.timestamp_sample, _last_signal);
	EXPECT_EQ(_manual_input.data_source, manual_control_setpoint_s::SOURCE_RC);
	EXPECT_FLOAT_EQ(_manual_input.roll, 1.f);
	EXPECT_NEAR(_manual_input.pitch, 0.f, 1e-6f);
	EXPECT_FLOAT_EQ(_manual_input.throttle, -1.f);
	EXPECT_NEAR(_manual_input.yaw, 0.f, 1e-6f);

	Srxl2TestManualControl manual_control;
	manual_control.processOnce(hrt_absolute_time());
	manual_control_setpoint_s selected {};
	ASSERT_TRUE(_manual_setpoint_sub.update(&selected));
	EXPECT_TRUE(selected.valid);
	EXPECT_EQ(selected.timestamp_sample, _last_signal);
	EXPECT_FLOAT_EQ(selected.roll, 1.f);
}

TEST_F(Srxl2RCUpdateTest, HeldAuxiliariesDoNotBlockFreshStickInput)
{
	establishSignal();
	ASSERT_FALSE(HasFatalFailure());
	Srxl2TestManualControl manual_control;
	manual_control.processOnce(hrt_absolute_time());
	manual_control_setpoint_s selected {};
	ASSERT_TRUE(_manual_setpoint_sub.update(&selected));
	ASSERT_TRUE(selected.valid);

	// Keep the serial bus active while CH5-7 are omitted for over 100 ms.
	for (unsigned i = 0; i < 6; ++i) {
		px4_usleep(20000);
		const auto now = hrt_absolute_time();
		receive(channels(0xf, 0, i % 2 == 0 ? 0x2aa0 : 0xd554), now, now);
		ASSERT_TRUE(_manual_input_sub.update(&_manual_input));
		EXPECT_TRUE(_manual_input.valid);
		EXPECT_EQ(_manual_input.timestamp_sample, now);
		EXPECT_FLOAT_EQ(_manual_input.roll, i % 2 == 0 ? -1.f : 1.f);
		manual_control.processOnce(now);
		ASSERT_TRUE(_manual_setpoint_sub.update(&selected));
		EXPECT_TRUE(selected.valid);
		EXPECT_EQ(selected.timestamp_sample, now);
	}

	EXPECT_EQ(_endpoint.channel_state(hrt_absolute_time()).fresh_mask, 0xfu);
}

TEST_F(Srxl2RCUpdateTest, ReceiverFailsafeStopsManualInputAndExpiresSelection)
{
	establishSignal();
	ASSERT_FALSE(HasFatalFailure());
	Srxl2TestManualControl manual_control;
	manual_control.processOnce(hrt_absolute_time());
	manual_control_setpoint_s selected {};
	ASSERT_TRUE(_manual_setpoint_sub.update(&selected));
	ASSERT_TRUE(selected.valid);

	const auto now = hrt_absolute_time();
	receive(channels(1, 1), now, now);
	rc_channels_s rc {};
	ASSERT_TRUE(_rc_channels_sub.update(&rc));
	EXPECT_TRUE(rc.signal_lost);
	EXPECT_EQ(rc.timestamp, _last_signal);
	EXPECT_FALSE(_manual_input_sub.update(&_manual_input));

	// ManualControl follows its normal COM_RC_LOSS_T policy; SRXL2 does not
	// bypass the selector or invent fresh input timestamps during failsafe.
	manual_control.processOnce(_last_signal + 500001);
	ASSERT_TRUE(_manual_setpoint_sub.update(&selected));
	EXPECT_FALSE(selected.valid);
}

TEST_F(Srxl2RCUpdateTest, FadePacketsCannotKeepStaleControlsAlive)
{
	establishSignal();
	ASSERT_FALSE(HasFatalFailure());
	Srxl2TestManualControl manual_control;
	manual_control.processOnce(hrt_absolute_time());
	manual_control_setpoint_s selected {};
	ASSERT_TRUE(_manual_setpoint_sub.update(&selected));
	ASSERT_TRUE(selected.valid);

	// Receiver serial traffic continues every 20 ms but reports no RF data.
	for (unsigned i = 1; i <= 6; ++i) {
		const auto now = _last_signal + i * 20000;
		receive(channels(0), now, now);
		EXPECT_FALSE(_manual_input_sub.update(&_manual_input));
	}

	rc_channels_s rc {};
	ASSERT_TRUE(_rc_channels_sub.update(&rc));
	EXPECT_TRUE(rc.signal_lost);
	EXPECT_EQ(rc.timestamp, _last_signal);
	manual_control.processOnce(_last_signal + 500001);
	ASSERT_TRUE(_manual_setpoint_sub.update(&selected));
	EXPECT_FALSE(selected.valid);
}
