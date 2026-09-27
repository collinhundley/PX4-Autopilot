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

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#define MSP_FRAME_START_SIZE 5
#define MSP_CRC_SIZE 1

class MspV1
{
public:
	static constexpr size_t MAX_PAYLOAD_SIZE = UINT8_MAX;
	static constexpr size_t TX_CAPACITY = 2048;

	// The optional callbacks have POSIX read/write semantics and allow deterministic transport tests.
	struct Io {
		ssize_t (*read)(int fd, void *buffer, size_t size, void *context);
		ssize_t (*write)(int fd, const void *buffer, size_t size, void *context);
		void *context;
	};

	explicit MspV1(int fd);
	MspV1(int fd, Io io);
	// Bind the freshly opened UART without a large temporary on the work-queue stack.
	void SetFileDescriptor(int fd) { _fd = fd; }
	int GetMessageSize(int message_type);

	// Success means the complete packet was queued. Call Flush() to transmit it.
	bool Send(uint8_t message_id, const void *payload);
	bool Send(uint8_t message_id, const void *payload, uint32_t payload_size);

	// One bounded write attempt. Pending bytes survive short writes and errors.
	// Returns 0 when drained, -EAGAIN while pending, or another negative errno.
	int Flush();
	size_t pending_bytes() const { return _tx_end - _tx_begin; }
	size_t free_tx_bytes() const { return TX_CAPACITY - pending_bytes(); }

	// Returns payload length, including zero, only after a complete valid frame.
	// Incomplete input returns -EAGAIN. The checksum is never copied to payload.
	int Receive(uint8_t *payload, size_t capacity, uint8_t *message_id);

private:
	static constexpr size_t RX_CHUNK_SIZE = 64;
	static constexpr size_t RX_BYTE_BUDGET = 512;
	enum class RxState : uint8_t { Start, Magic, Direction, Length, Command, Payload, Checksum };

	static ssize_t Read(int fd, void *buffer, size_t size, void *context);
	static ssize_t Write(int fd, const void *buffer, size_t size, void *context);

	int _fd{-1};
	Io _io;
	uint8_t _tx_buffer[TX_CAPACITY] {};
	size_t _tx_begin{0};
	size_t _tx_end{0};

	uint8_t _rx_buffer[RX_CHUNK_SIZE] {};
	size_t _rx_begin{0};
	size_t _rx_end{0};
	uint8_t _rx_payload[MAX_PAYLOAD_SIZE] {};
	RxState _rx_state{RxState::Start};
	uint8_t _rx_size{0};
	uint8_t _rx_id{0};
	uint8_t _rx_crc{0};
	size_t _rx_position{0};
};
