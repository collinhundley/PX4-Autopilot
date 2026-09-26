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

#pragma once

#include <cstddef>
#include <cstdint>
#include <sys/types.h>
#include <termios.h>

/** Opt-in STM32H7 half-duplex transport; unsupported targets fail closed. */
class Srxl2Transport
{
public:
	struct Timing {
		uint64_t first_sequence{};
		uint64_t end_sequence{};
		uint64_t start_lower_bound_us{};
		uint64_t idle_observed_us{};
		bool valid{};
	};

	struct RxChunk {
		uint64_t first_sequence{};
		uint64_t end_sequence{};
		Timing timing{};
	};

	enum class TxResult { Sent, Busy, Stale, Unsupported, Error };

	struct Counters {
		uint32_t rx_bytes{};
		uint32_t tx_packets{};
		uint32_t busy{};
		uint32_t stale{};
		uint32_t errors{};
		uint32_t rx_overruns{};
	};

	Srxl2Transport() = default;
	~Srxl2Transport() { close(); }
	Srxl2Transport(const Srxl2Transport &) = delete;
	Srxl2Transport &operator=(const Srxl2Transport &) = delete;

	bool open(const char *device);
	void close();
	bool isOpen() const { return _fd >= 0 && _timed_enabled; }
	ssize_t read(uint8_t *buffer, size_t capacity, RxChunk &chunk);
	bool packetTiming(uint64_t first_sequence, uint64_t end_sequence, Timing &timing);
	TxResult tryTransmit(const uint8_t *buffer, size_t length, uint64_t request_first_sequence,
			     uint64_t request_end_sequence, bool handshake = false);
	bool txBusy();
	const Counters &counters() const { return _counters; }
	int lastError() const { return _last_error; }

private:
	static uint64_t clockUs();
	int _fd{-1};
	int _last_error{};
	bool _timed_enabled{};
	bool _have_original_termios{};
	termios _original_termios{};
	Counters _counters{};
};
