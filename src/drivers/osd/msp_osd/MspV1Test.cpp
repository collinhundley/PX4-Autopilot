/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "MspV1.hpp"

#include <gtest/gtest.h>
#include <algorithm>
#include <errno.h>
#include <string.h>
#include <vector>

namespace
{
std::vector<uint8_t> frame(uint8_t id, const std::vector<uint8_t> &payload, uint8_t direction = '<')
{
	std::vector<uint8_t> result {'$', 'M', direction, static_cast<uint8_t>(payload.size()), id};
	uint8_t crc = payload.size() ^ id;

	for (const uint8_t byte : payload) {
		result.push_back(byte);
		crc ^= byte;
	}

	result.push_back(crc);
	return result;
}

struct Transport {
	std::vector<uint8_t> input;
	std::vector<uint8_t> output;
	std::vector<int> writes;
	size_t read_position{0};
	size_t write_position{0};
	size_t read_limit{SIZE_MAX};
	int read_error{0};
	bool eof{false};

	MspV1::Io io() { return {Read, Write, this}; }

	static ssize_t Read(int, void *buffer, size_t size, void *context)
	{
		auto &transport = *static_cast<Transport *>(context);

		if (transport.read_error != 0) {
			errno = transport.read_error;
			transport.read_error = 0;
			return -1;
		}

		const size_t count = std::min(std::min(size, transport.read_limit),
					      transport.input.size() - transport.read_position);

		if (count == 0) {
			errno = EAGAIN;
			return transport.eof ? 0 : -1;
		}

		memcpy(buffer, transport.input.data() + transport.read_position, count);
		transport.read_position += count;
		return count;
	}

	static ssize_t Write(int, const void *buffer, size_t size, void *context)
	{
		auto &transport = *static_cast<Transport *>(context);
		int result = size;

		if (transport.write_position < transport.writes.size()) {
			result = transport.writes[transport.write_position++];
		}

		if (result < 0) {
			errno = -result;
			return -1;
		}

		const size_t count = std::min(size, static_cast<size_t>(result));
		const auto *bytes = static_cast<const uint8_t *>(buffer);
		transport.output.insert(transport.output.end(), bytes, bytes + count);
		return count;
	}
};
}

TEST(MspV1Test, SendsCorrectFrameOnlyWhenFlushed)
{
	Transport transport;
	MspV1 msp(-1, transport.io());
	const std::vector<uint8_t> payload {0, 1, 0xff, '$'};
	ASSERT_TRUE(msp.Send(182, payload.data(), payload.size()));
	EXPECT_TRUE(transport.output.empty());
	EXPECT_EQ(msp.pending_bytes(), payload.size() + 6);
	EXPECT_EQ(msp.Flush(), 0);
	EXPECT_EQ(transport.output, frame(182, payload, '>'));
	EXPECT_EQ(msp.pending_bytes(), 0u);
	EXPECT_EQ(msp.free_tx_bytes(), MspV1::TX_CAPACITY);
}

TEST(MspV1Test, MaximumPayloadAndEmptyPayloadAreValid)
{
	Transport transport;
	MspV1 msp(-1, transport.io());
	const std::vector<uint8_t> payload(255, 0xa5);
	ASSERT_TRUE(msp.Send(1, payload.data(), payload.size()));
	ASSERT_TRUE(msp.Send(2, nullptr, 0));
	ASSERT_EQ(msp.Flush(), 0);
	auto expected = frame(1, payload, '>');
	const auto empty = frame(2, {}, '>');
	expected.insert(expected.end(), empty.begin(), empty.end());
	EXPECT_EQ(transport.output, expected);
}

TEST(MspV1Test, RejectsOversizeNullAndUnknownPayloadsWithoutChangingQueue)
{
	Transport transport;
	MspV1 msp(-1, transport.io());
	uint8_t byte = 0;
	EXPECT_FALSE(msp.Send(1, &byte, 256));
	EXPECT_FALSE(msp.Send(1, &byte, UINT32_MAX));
	EXPECT_FALSE(msp.Send(1, nullptr, 1));
	EXPECT_FALSE(msp.Send(0xff, &byte));
	EXPECT_EQ(msp.pending_bytes(), 0u);
	EXPECT_EQ(msp.GetMessageSize(0xff), -EINVAL);
	EXPECT_GT(msp.GetMessageSize(101), 0);
}

TEST(MspV1Test, QueueRejectsWholePacketWithoutPartialEnqueue)
{
	Transport transport;
	MspV1 msp(-1, transport.io());
	const std::vector<uint8_t> payload(255, 7);

	for (unsigned i = 0; i < 7; ++i) {
		ASSERT_TRUE(msp.Send(i, payload.data(), payload.size()));
	}

	const size_t pending = msp.pending_bytes();
	EXPECT_FALSE(msp.Send(8, payload.data(), payload.size()));
	EXPECT_EQ(msp.pending_bytes(), pending);
	EXPECT_EQ(msp.Flush(), 0);
	EXPECT_EQ(transport.output.size(), pending);
}

TEST(MspV1Test, ShortWritesAndBackpressurePreservePacketOrder)
{
	Transport transport;
	transport.writes = {3, -EAGAIN, 1, -EINTR, 2};
	MspV1 msp(-1, transport.io());
	const std::vector<uint8_t> first {1, 2, 3};
	const std::vector<uint8_t> second {4, 5};
	ASSERT_TRUE(msp.Send(7, first.data(), first.size()));
	ASSERT_EQ(msp.Flush(), -EAGAIN);
	ASSERT_TRUE(msp.Send(8, second.data(), second.size()));

	for (unsigned i = 0; i < 4; ++i) {
		EXPECT_EQ(msp.Flush(), -EAGAIN);
	}

	ASSERT_EQ(msp.Flush(), 0);
	auto expected = frame(7, first, '>');
	const auto next = frame(8, second, '>');
	expected.insert(expected.end(), next.begin(), next.end());
	EXPECT_EQ(transport.output, expected);
}

TEST(MspV1Test, ReusesDrainedPrefixWithoutCorruptingPendingData)
{
	Transport transport;
	transport.writes = {1000};
	MspV1 msp(-1, transport.io());
	const std::vector<uint8_t> payload(255, 0xa5);
	std::vector<uint8_t> expected;

	for (unsigned i = 0; i < 9; ++i) {
		if (i == 7) {
			ASSERT_EQ(msp.Flush(), -EAGAIN);
		}

		ASSERT_TRUE(msp.Send(i, payload.data(), payload.size()));
		const auto packet = frame(i, payload, '>');
		expected.insert(expected.end(), packet.begin(), packet.end());
	}

	ASSERT_EQ(msp.Flush(), 0);
	EXPECT_EQ(transport.output, expected);
}

TEST(MspV1Test, WriteErrorsDoNotDiscardPendingBytes)
{
	Transport transport;
	transport.writes = {-EIO, 0};
	MspV1 msp(-1, transport.io());
	ASSERT_TRUE(msp.Send(1, nullptr, 0));
	EXPECT_EQ(msp.Flush(), -EIO);
	EXPECT_EQ(msp.pending_bytes(), 6u);
	EXPECT_EQ(msp.Flush(), -EIO);
	EXPECT_EQ(msp.pending_bytes(), 6u);
	EXPECT_EQ(msp.Flush(), 0);
	EXPECT_EQ(transport.output, frame(1, {}, '>'));
}

TEST(MspV1Test, ReceivesEveryHeaderPayloadAndChecksumFragment)
{
	Transport transport;
	MspV1 msp(-1, transport.io());
	const auto input = frame(89, {1, 2, 3, 4});
	uint8_t payload[4] {0xa5, 0xa5, 0xa5, 0xa5};
	uint8_t id = 0xa5;

	for (size_t i = 0; i < input.size(); ++i) {
		transport.input.push_back(input[i]);
		const int result = msp.Receive(payload, sizeof(payload), &id);

		if (i + 1 < input.size()) {
			EXPECT_EQ(result, -EAGAIN);
			EXPECT_EQ(id, 0xa5);
			EXPECT_EQ(payload[0], 0xa5);

		} else {
			EXPECT_EQ(result, 4);
		}
	}

	EXPECT_EQ(id, 89);
	EXPECT_EQ(std::vector<uint8_t>(payload, payload + 4), (std::vector<uint8_t> {1, 2, 3, 4}));
}

TEST(MspV1Test, MaximumIncomingPayloadNeverCopiesChecksum)
{
	Transport transport;
	const std::vector<uint8_t> data(255, 0x42);
	transport.input = frame(7, data);
	MspV1 msp(-1, transport.io());
	uint8_t guarded[257];
	memset(guarded, 0xa5, sizeof(guarded));
	uint8_t id = 0;
	ASSERT_EQ(msp.Receive(guarded + 1, 255, &id), 255);
	EXPECT_EQ(guarded[0], 0xa5);
	EXPECT_EQ(guarded[256], 0xa5);
	EXPECT_EQ(std::vector<uint8_t>(guarded + 1, guarded + 256), data);
	EXPECT_EQ(id, 7);
}

TEST(MspV1Test, RejectsBadChecksumAndResumesAtNextBufferedFrame)
{
	Transport transport;
	transport.input = frame(7, {1, 2});
	transport.input.back() ^= 1;
	const auto good = frame(8, {3});
	transport.input.insert(transport.input.end(), good.begin(), good.end());
	MspV1 msp(-1, transport.io());
	uint8_t payload[2] {0xa5, 0xa5};
	uint8_t id = 0xa5;
	EXPECT_EQ(msp.Receive(payload, sizeof(payload), &id), -EBADMSG);
	EXPECT_EQ(payload[0], 0xa5);
	EXPECT_EQ(id, 0xa5);
	EXPECT_EQ(msp.Receive(payload, sizeof(payload), &id), 1);
	EXPECT_EQ(payload[0], 3);
	EXPECT_EQ(id, 8);
}

TEST(MspV1Test, TooSmallDestinationConsumesFrameWithoutCopying)
{
	Transport transport;
	transport.input = frame(7, {1, 2});
	const auto next = frame(8, {});
	transport.input.insert(transport.input.end(), next.begin(), next.end());
	MspV1 msp(-1, transport.io());
	uint8_t byte = 0xa5;
	uint8_t id = 0xa5;
	EXPECT_EQ(msp.Receive(&byte, 1, &id), -EMSGSIZE);
	EXPECT_EQ(byte, 0xa5);
	EXPECT_EQ(id, 0xa5);
	EXPECT_EQ(msp.Receive(nullptr, 0, &id), 0);
	EXPECT_EQ(id, 8);
}

TEST(MspV1Test, RejectsBadArgumentsWithoutConsumingInput)
{
	Transport transport;
	transport.input = frame(7, {});
	MspV1 msp(-1, transport.io());
	uint8_t id = 0;
	EXPECT_EQ(msp.Receive(nullptr, 1, &id), -EINVAL);
	EXPECT_EQ(msp.Receive(nullptr, 0, nullptr), -EINVAL);
	EXPECT_EQ(transport.read_position, 0u);
	EXPECT_EQ(msp.Receive(nullptr, 0, &id), 0);
}

TEST(MspV1Test, RecoversFromNoiseRepeatedPreambleAndWrongDirection)
{
	Transport transport;
	transport.input = {0, '$', '$', 'M', '>', 0, 1, 1, '$', 'x', '$'};
	const auto good = frame(8, {7});
	transport.input.insert(transport.input.end(), good.begin(), good.end());
	MspV1 msp(-1, transport.io());
	uint8_t byte = 0;
	uint8_t id = 0;
	EXPECT_EQ(msp.Receive(&byte, 1, &id), 1);
	EXPECT_EQ(byte, 7);
	EXPECT_EQ(id, 8);
}

TEST(MspV1Test, BoundsWorkWhenInputContainsNoFrames)
{
	Transport transport;
	transport.input.resize(2048, 0);
	MspV1 msp(-1, transport.io());
	uint8_t id = 0;
	EXPECT_EQ(msp.Receive(nullptr, 0, &id), -EAGAIN);
	EXPECT_EQ(transport.read_position, 512u);
}

TEST(MspV1Test, DistinguishesEofAndRetainsFragmentsAcrossInterruptedRead)
{
	Transport transport;
	MspV1 msp(-1, transport.io());
	uint8_t id = 0;
	transport.input = {'$', 'M'};
	EXPECT_EQ(msp.Receive(nullptr, 0, &id), -EAGAIN);
	transport.read_error = EINTR;
	EXPECT_EQ(msp.Receive(nullptr, 0, &id), -EAGAIN);
	transport.read_error = EIO;
	EXPECT_EQ(msp.Receive(nullptr, 0, &id), -EIO);
	transport.input.insert(transport.input.end(), {'<', 0, 7, 7});
	EXPECT_EQ(msp.Receive(nullptr, 0, &id), 0);
	EXPECT_EQ(id, 7);
	transport.eof = true;
	EXPECT_EQ(msp.Receive(nullptr, 0, &id), -EIO);
}
