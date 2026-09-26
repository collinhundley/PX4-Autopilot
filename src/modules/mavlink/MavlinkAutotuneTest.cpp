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

#include "MavlinkAutotune.hpp"

#include <gtest/gtest.h>

using namespace time_literals;
using Status = autotune_attitude_control_status_s;
using Ack = vehicle_command_ack_s;

class MavlinkAutotuneTest : public ::testing::Test
{
protected:
	MavlinkAutotune handler;
	vehicle_status_s vehicle{};
	Status status{};
	static constexpr hrt_abstime START = 10_s;

	void SetUp() override
	{
		vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.vehicle_type = vehicle.vehicle_type;
		status.timestamp_start = START;
		status.timestamp = START;
		status.state = Status::STATE_INIT;
	}

	MavlinkAutotune::Response request(hrt_abstime now, const Status *sample = nullptr)
	{
		vehicle.timestamp = now;
		return handler.request(now, &vehicle, sample, true);
	}
};

TEST_F(MavlinkAutotuneTest, FreshBootDispatchesWithoutPriorStatus)
{
	const auto response = request(START);
	EXPECT_EQ(response.result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
	EXPECT_EQ(response.progress, 0);
	EXPECT_TRUE(response.publish_command);

	status.timestamp_start = START + 100_ms;
	status.timestamp = START + 100_ms;
	const auto initialized = request(START + 1_s, &status);
	EXPECT_EQ(initialized.result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
	EXPECT_FALSE(initialized.publish_command);
}

TEST_F(MavlinkAutotuneTest, MissingModuleFailsWithinThreeSecondsAndPollsNeverRestartIt)
{
	ASSERT_TRUE(request(START).publish_command);

	for (unsigned seconds = 1; seconds < 60; ++seconds) {
		const auto response = request(START + seconds * 1_s);
		EXPECT_FALSE(response.publish_command);
		EXPECT_EQ(response.result, seconds < 3 ? Ack::VEHICLE_CMD_RESULT_IN_PROGRESS : Ack::VEHICLE_CMD_RESULT_FAILED);
	}

	const auto retry = request(START + 63_s);
	EXPECT_TRUE(retry.publish_command);
	EXPECT_EQ(retry.result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
}

TEST_F(MavlinkAutotuneTest, PollsTrackTheActualAttemptWithoutDispatchingAgain)
{
	ASSERT_TRUE(request(START).publish_command);
	status.timestamp_start = START + 100_ms;
	status.state = Status::STATE_ROLL;

	for (unsigned seconds = 1; seconds < 20; ++seconds) {
		status.timestamp = START + seconds * 1_s;
		const auto response = request(status.timestamp, &status);
		EXPECT_EQ(response.result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
		EXPECT_EQ(response.progress, 20);
		EXPECT_FALSE(response.publish_command);
	}
}

TEST_F(MavlinkAutotuneTest, WrongTunerStatusCannotSatisfyTheHandshake)
{
	ASSERT_TRUE(request(START).publish_command);
	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	status.timestamp_start = START + 100_ms;
	status.state = Status::STATE_COMPLETE;

	for (unsigned seconds = 1; seconds <= 3; ++seconds) {
		status.timestamp = START + seconds * 1_s;
		const auto response = request(status.timestamp, &status);
		EXPECT_FALSE(response.publish_command);
		EXPECT_EQ(response.result, seconds < 3 ? Ack::VEHICLE_CMD_RESULT_IN_PROGRESS : Ack::VEHICLE_CMD_RESULT_FAILED);
		EXPECT_EQ(response.progress, 0);
	}
}

TEST_F(MavlinkAutotuneTest, EarlierTerminalIsNotTheNewAttemptsOutcome)
{
	status.state = Status::STATE_COMPLETE;
	const auto start = request(START + 2_s, &status); // Retained but stale terminal sample.
	ASSERT_TRUE(start.publish_command);
	status.timestamp = START + 3_s; // Even a republished old attempt cannot satisfy the request.
	const auto poll = request(START + 3_s, &status);
	EXPECT_EQ(poll.result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
	EXPECT_EQ(poll.progress, 0);
	EXPECT_FALSE(poll.publish_command);
	EXPECT_EQ(request(START + 5_s, &status).result, Ack::VEHICLE_CMD_RESULT_FAILED);
}

TEST_F(MavlinkAutotuneTest, TerminalFromOtherTunerDoesNotBlockFirstStart)
{
	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	status.state = Status::STATE_COMPLETE;
	const auto response = request(START, &status);
	EXPECT_TRUE(response.publish_command);
	EXPECT_EQ(response.result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
	EXPECT_EQ(response.progress, 0);
}

TEST_F(MavlinkAutotuneTest, ActiveOtherTunerIsRejected)
{
	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	status.state = Status::STATE_ROLL_AMPLITUDE_DETECTION;
	const auto response = request(START, &status);
	EXPECT_FALSE(response.publish_command);
	EXPECT_EQ(response.result, Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
}

TEST_F(MavlinkAutotuneTest, ExistingMissionOrAuxAttemptIsAdopted)
{
	status.state = Status::STATE_PITCH;
	const auto response = request(START, &status);
	EXPECT_FALSE(response.publish_command);
	EXPECT_EQ(response.result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
	EXPECT_EQ(response.progress, 40);
}

TEST_F(MavlinkAutotuneTest, PendingRequestAdoptsConcurrentMissionOrAuxAttempt)
{
	// Another trigger acquired the channel before our request, but its INIT was
	// not yet visible when the receiver copied the topic.
	ASSERT_TRUE(request(START).publish_command);
	status.timestamp_start = START - 100_ms;
	status.timestamp = START - 1_ms;
	status.state = Status::STATE_ROLL;

	// A retained pre-request sample does not establish that the owner is alive.
	const auto retained = request(START + 10_ms, &status);
	EXPECT_EQ(retained.result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
	EXPECT_EQ(retained.progress, 0);
	EXPECT_FALSE(retained.publish_command);

	status.timestamp = START + 100_ms;
	const auto heartbeat = request(START + 100_ms, &status);
	EXPECT_EQ(heartbeat.result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
	EXPECT_EQ(heartbeat.progress, 20);
	EXPECT_FALSE(heartbeat.publish_command);
}

TEST_F(MavlinkAutotuneTest, MissingEstablishedStatusFails)
{
	status.state = Status::STATE_ROLL;
	ASSERT_EQ(request(START, &status).progress, 20);
	const auto response = request(START + 1_s);
	EXPECT_FALSE(response.publish_command);
	EXPECT_EQ(response.result, Ack::VEHICLE_CMD_RESULT_FAILED);
}

TEST_F(MavlinkAutotuneTest, StaleEstablishedStatusFails)
{
	ASSERT_EQ(request(START, &status).result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
	const auto response = request(START + 1_s + 1_us, &status);
	EXPECT_FALSE(response.publish_command);
	EXPECT_EQ(response.result, Ack::VEHICLE_CMD_RESULT_FAILED);
}

TEST_F(MavlinkAutotuneTest, TransitionRejectsFirstStartAndFailsExistingAttempt)
{
	vehicle.in_transition_mode = true;
	const auto rejected = request(START);
	EXPECT_FALSE(rejected.publish_command);
	EXPECT_EQ(rejected.result, Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
	vehicle.in_transition_mode = false;
	ASSERT_TRUE(request(START + 1_s).publish_command);
	vehicle.in_transition_mode = true;
	const auto failed = request(START + 2_s);
	EXPECT_FALSE(failed.publish_command);
	EXPECT_EQ(failed.result, Ack::VEHICLE_CMD_RESULT_FAILED);
}

TEST_F(MavlinkAutotuneTest, VehicleTypeChangeFailsAttempt)
{
	ASSERT_TRUE(request(START).publish_command);
	vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	const auto response = request(START + 1_s);
	EXPECT_FALSE(response.publish_command);
	EXPECT_EQ(response.result, Ack::VEHICLE_CMD_RESULT_FAILED);
}

TEST_F(MavlinkAutotuneTest, MissingOrStaleVehicleStatusCannotStart)
{
	EXPECT_EQ(handler.request(START, nullptr, nullptr, true).result,
		  Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
	vehicle.timestamp = START - 3_s;
	const auto response = handler.request(START, &vehicle, nullptr, true);
	EXPECT_FALSE(response.publish_command);
	EXPECT_EQ(response.result, Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
}

TEST_F(MavlinkAutotuneTest, MissingBuildSupportIsUnsupported)
{
	vehicle.timestamp = START;
	const auto response = handler.request(START, &vehicle, nullptr, false);
	EXPECT_FALSE(response.publish_command);
	EXPECT_EQ(response.result, Ack::VEHICLE_CMD_RESULT_UNSUPPORTED);
}

TEST_F(MavlinkAutotuneTest, CompletionAndReturnToIdlePermitSecondAttempt)
{
	ASSERT_TRUE(request(START).publish_command);
	status.timestamp_start = START + 100_ms;
	status.timestamp = START + 1_s;
	status.state = Status::STATE_COMPLETE;
	const auto complete = request(status.timestamp, &status);
	EXPECT_FALSE(complete.publish_command);
	EXPECT_EQ(complete.result, Ack::VEHICLE_CMD_RESULT_ACCEPTED);
	EXPECT_EQ(complete.progress, 100);

	// Identical requests during cooldown must not report the old success as a new success.
	status.timestamp = START + 2_s;
	const auto cooldown = request(status.timestamp, &status);
	EXPECT_FALSE(cooldown.publish_command);
	EXPECT_EQ(cooldown.result, Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);

	status.state = Status::STATE_IDLE;
	status.timestamp = START + 3_s;
	const auto second = request(status.timestamp, &status);
	EXPECT_TRUE(second.publish_command);
	EXPECT_EQ(second.result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);

	// IDLE from the previous attempt cannot satisfy the second handshake.
	status.timestamp = START + 4_s;
	EXPECT_EQ(request(status.timestamp, &status).result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
	status.timestamp_start = START + 3_s + 100_ms;
	status.state = Status::STATE_PITCH;
	EXPECT_EQ(request(status.timestamp, &status).progress, 40);
}

TEST_F(MavlinkAutotuneTest, FailureAndReturnToIdlePermitSecondAttempt)
{
	ASSERT_TRUE(request(START).publish_command);
	status.timestamp_start = START + 100_ms;
	status.timestamp = START + 1_s;
	status.state = Status::STATE_FAIL;
	EXPECT_EQ(request(status.timestamp, &status).result, Ack::VEHICLE_CMD_RESULT_FAILED);
	status.timestamp = START + 3_s;
	status.state = Status::STATE_IDLE;
	EXPECT_TRUE(request(status.timestamp, &status).publish_command);
}

TEST_F(MavlinkAutotuneTest, IdleCannotFabricateInitializationOfAnObservedAttempt)
{
	ASSERT_EQ(request(START, &status).result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
	status.timestamp = START + 1_s;
	status.state = Status::STATE_IDLE;
	EXPECT_EQ(request(status.timestamp, &status).result, Ack::VEHICLE_CMD_RESULT_FAILED);
}

TEST_F(MavlinkAutotuneTest, ProgressMappingPreservesTheQgcInterface)
{
	struct Expected {
		uint8_t state;
		uint8_t progress;
	};
	const Expected expected[] {
		{Status::STATE_INIT, 0},
		{Status::STATE_ROLL_AMPLITUDE_DETECTION, 20},
		{Status::STATE_ROLL, 20},
		{Status::STATE_ROLL_PAUSE, 20},
		{Status::STATE_PITCH_AMPLITUDE_DETECTION, 40},
		{Status::STATE_PITCH, 40},
		{Status::STATE_PITCH_PAUSE, 40},
		{Status::STATE_YAW_AMPLITUDE_DETECTION, 60},
		{Status::STATE_YAW, 60},
		{Status::STATE_YAW_PAUSE, 60},
		{Status::STATE_VERIFICATION, 80},
		{Status::STATE_APPLY, 85},
		{Status::STATE_TEST, 90},
		{Status::STATE_WAIT_FOR_DISARM, 95},
	};

	for (const auto &entry : expected) {
		status.state = entry.state;
		const auto response = request(START, &status);
		EXPECT_FALSE(response.publish_command);
		EXPECT_EQ(response.result, Ack::VEHICLE_CMD_RESULT_IN_PROGRESS);
		EXPECT_EQ(response.progress, entry.progress);
	}
}
