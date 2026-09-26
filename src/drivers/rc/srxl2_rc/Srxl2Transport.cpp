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

#include "Srxl2Transport.hpp"

#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#if defined(__PX4_NUTTX) && defined(CONFIG_STM32H7_SERIAL_TIMED_HALF_DUPLEX)
# include <nuttx/serial/timed_halfduplex.h>
# define SRXL2_TIMED_UART_SUPPORTED 1
#endif

namespace
{
constexpr uint32_t Baudrate = 115200;
constexpr uint32_t MinimumFramePeriodUs = 11000;
constexpr uint32_t HandshakeMaxReplyAgeUs = 1000;
constexpr uint32_t TelemetryMaxReplyAgeUs = 2000;
// This extra budget is in addition to conservative RX-start capture and the
// two post-TX idle characters. Hardware acceptance must verify timing margins.
constexpr uint32_t TurnaroundMarginUs = 100;
constexpr size_t MaxPacketSize = 80;
constexpr size_t MaxReadSize = 2 * MaxPacketSize;

#ifdef SRXL2_TIMED_UART_SUPPORTED
Srxl2Transport::Timing timingFromStatus(const serial_thdx_status_s &status)
{
	Srxl2Transport::Timing timing{};
	timing.first_sequence = status.first_sequence;
	timing.end_sequence = status.end_sequence;
	timing.start_lower_bound_us = status.start_lower_bound_us;
	timing.idle_observed_us = status.idle_observed_us;
	timing.valid = status.timing_valid;
	return timing;
}
#endif
} // namespace

uint64_t Srxl2Transport::clockUs()
{
	return hrt_absolute_time();
}

bool Srxl2Transport::open(const char *device)
{
	if (isOpen()) {
		return true;
	}

#ifdef SRXL2_TIMED_UART_SUPPORTED

	if (device == nullptr) {
		_last_error = EINVAL;
		return false;
	}

	_fd = ::open(device, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if (_fd < 0) {
		_last_error = errno;
		return false;
	}

	// Claim before modifying baud, flow control or pins. Manual starts on a
	// UART already owned by another driver must leave that driver untouched.
	if (ioctl(_fd, TIOCEXCL, 0) != 0) {
		_last_error = errno;
		close();
		return false;
	}

	if (tcgetattr(_fd, &_original_termios) != 0) {
		_last_error = errno;
		close();
		return false;
	}

	_have_original_termios = true;
	termios config{};
	config.c_cflag = CS8 | CLOCAL | CREAD;
	config.c_cc[VMIN] = 0;
	config.c_cc[VTIME] = 0;

	if (cfsetispeed(&config, Baudrate) != 0 || cfsetospeed(&config, Baudrate) != 0
	    || tcsetattr(_fd, TCSANOW, &config) != 0) {
		_last_error = errno;
		close();
		return false;
	}

	serial_thdx_config_s timed_config{};
	timed_config.clock_us = clockUs;
	timed_config.enable = true;

	if (ioctl(_fd, TIOCSTIMEDHDX, reinterpret_cast<unsigned long>(&timed_config)) != 0) {
		_last_error = errno;
		close();
		return false;
	}

	_timed_enabled = true;
	_last_error = 0;
	return true;
#else
	(void)device;
	_last_error = ENOTSUP;
	return false;
#endif
}

void Srxl2Transport::close()
{
	if (_fd < 0) {
		return;
	}

#ifdef SRXL2_TIMED_UART_SUPPORTED

	if (_timed_enabled) {
		serial_thdx_config_s config{};
		ioctl(_fd, TIOCSTIMEDHDX, reinterpret_cast<unsigned long>(&config));
		_timed_enabled = false;
	}

#endif

	if (_have_original_termios) {
		tcsetattr(_fd, TCSANOW, &_original_termios);
		_have_original_termios = false;
	}

	::close(_fd);
	_fd = -1;
}

ssize_t Srxl2Transport::read(uint8_t *buffer, size_t capacity, RxChunk &chunk)
{
	chunk = {};

	if (!isOpen() || buffer == nullptr || capacity == 0) {
		_last_error = !isOpen() ? EBADF : EINVAL;
		return -1;
	}

#ifdef SRXL2_TIMED_UART_SUPPORTED
	serial_thdx_read_s request {};
	request.buffer = buffer;
	request.capacity = capacity > MaxReadSize ? MaxReadSize : capacity;

	if (ioctl(_fd, TIOCGRXTIMEDHDX, reinterpret_cast<unsigned long>(&request)) != 0) {
		_last_error = errno;
		_counters.errors++;
		return -1;
	}

	chunk.first_sequence = request.first_sequence;
	chunk.end_sequence = request.end_sequence;
	chunk.timing = timingFromStatus(request.status);
	_counters.rx_bytes += request.length;
	_counters.rx_overruns = request.status.rx_overruns;
	return request.length;
#else
	_last_error = ENOTSUP;
	return -1;
#endif
}

bool Srxl2Transport::packetTiming(uint64_t first_sequence, uint64_t end_sequence, Timing &timing)
{
	timing = {};
#ifdef SRXL2_TIMED_UART_SUPPORTED
	serial_thdx_status_s status {};

	if (!isOpen() || ioctl(_fd, TIOCGTIMEDHDX, reinterpret_cast<unsigned long>(&status)) != 0) {
		return false;
	}

	timing = timingFromStatus(status);
	return timing.valid && first_sequence >= timing.first_sequence
	       && end_sequence <= timing.end_sequence && first_sequence < end_sequence;
#else
	(void)first_sequence;
	(void)end_sequence;
	return false;
#endif
}

Srxl2Transport::TxResult Srxl2Transport::tryTransmit(const uint8_t *buffer, size_t length,
		uint64_t request_first_sequence, uint64_t request_end_sequence, bool handshake)
{
#ifdef SRXL2_TIMED_UART_SUPPORTED

	if (!isOpen() || buffer == nullptr || length == 0 || length > MaxPacketSize) {
		_last_error = !isOpen() ? EBADF : EINVAL;
		_counters.errors++;
		return TxResult::Error;
	}

	serial_thdx_tx_s packet{};
	packet.buffer = buffer;
	packet.length = length;
	packet.request_first_sequence = request_first_sequence;
	packet.request_end_sequence = request_end_sequence;
	packet.frame_period_us = MinimumFramePeriodUs;
	packet.max_idle_age_us = handshake ? HandshakeMaxReplyAgeUs : TelemetryMaxReplyAgeUs;
	packet.guard_us = TurnaroundMarginUs;

	if (ioctl(_fd, TIOCSTXTIMEDHDX, reinterpret_cast<unsigned long>(&packet)) == 0) {
		_counters.tx_packets++;
		return TxResult::Sent;
	}

	_last_error = errno;

	switch (errno) {
	case EAGAIN:
	case EBUSY:
		_counters.busy++;
		return TxResult::Busy;

	case ESTALE:
	case ENODATA:
	case ETIMEDOUT:
		_counters.stale++;
		return TxResult::Stale;

	case ENOTTY:
	case ENOTSUP:
		_counters.errors++;
		return TxResult::Unsupported;

	default:
		_counters.errors++;
		return TxResult::Error;
	}

#else
	(void)buffer;
	(void)length;
	(void)request_first_sequence;
	(void)request_end_sequence;
	(void)handshake;
	_last_error = ENOTSUP;
	return TxResult::Unsupported;
#endif
}

bool Srxl2Transport::txBusy()
{
#ifdef SRXL2_TIMED_UART_SUPPORTED
	serial_thdx_status_s status {};
	return isOpen() && ioctl(_fd, TIOCGTIMEDHDX, reinterpret_cast<unsigned long>(&status)) == 0 && status.tx_busy;
#else
	return false;
#endif
}
