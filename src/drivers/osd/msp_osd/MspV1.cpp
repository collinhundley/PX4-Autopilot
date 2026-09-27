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
#include "msp_defines.h"

#include <errno.h>
#include <string.h>
#include <unistd.h>

namespace
{
struct MessageDescriptor {
	uint8_t message_id;
	uint8_t message_size;
};

constexpr MessageDescriptor message_descriptors[] = {
	{MSP_OSD_CONFIG, sizeof(msp_osd_config_t)},
	{MSP_NAME, sizeof(msp_name_t)},
	{MSP_ANALOG, sizeof(msp_analog_t)},
	{MSP_STATUS, sizeof(msp_status_BF_t)},
	{MSP_BATTERY_STATE, sizeof(msp_battery_state_t)},
	{MSP_RAW_GPS, sizeof(msp_raw_gps_t)},
	{MSP_ATTITUDE, sizeof(msp_attitude_t)},
	{MSP_ALTITUDE, sizeof(msp_altitude_t)},
	{MSP_COMP_GPS, sizeof(msp_comp_gps_t)},
	{MSP_ESC_SENSOR_DATA, sizeof(msp_esc_sensor_data_dji_t)},
	{MSP_MOTOR_TELEMETRY, sizeof(msp_motor_telemetry_t)},
	{MSP_RC, sizeof(msp_rc_t)},
};
}

MspV1::MspV1(int fd) : MspV1(fd, Io{Read, Write, nullptr})
{
}

MspV1::MspV1(int fd, Io io) : _fd(fd), _io(io)
{
}

ssize_t MspV1::Read(int fd, void *buffer, size_t size, void *)
{
	return read(fd, buffer, size);
}

ssize_t MspV1::Write(int fd, const void *buffer, size_t size, void *)
{
	return write(fd, buffer, size);
}

int MspV1::GetMessageSize(int message_type)
{
	for (const auto &descriptor : message_descriptors) {
		if (message_type == descriptor.message_id) {
			return descriptor.message_size;
		}
	}

	return -EINVAL;
}

bool MspV1::Send(uint8_t message_id, const void *payload)
{
	const int size = GetMessageSize(message_id);
	return size >= 0 && Send(message_id, payload, size);
}

bool MspV1::Send(uint8_t message_id, const void *payload, uint32_t payload_size)
{
	if (payload_size > MAX_PAYLOAD_SIZE || (payload_size != 0 && payload == nullptr)) {
		return false;
	}

	const size_t packet_size = MSP_FRAME_START_SIZE + payload_size + MSP_CRC_SIZE;

	if (packet_size > free_tx_bytes()) {
		return false;
	}

	if (_tx_end + packet_size > TX_CAPACITY) {
		const size_t pending = pending_bytes();
		memmove(_tx_buffer, _tx_buffer + _tx_begin, pending);
		_tx_begin = 0;
		_tx_end = pending;
	}

	uint8_t *packet = _tx_buffer + _tx_end;
	packet[0] = '$';
	packet[1] = 'M';
	packet[2] = '>';
	packet[3] = payload_size;
	packet[4] = message_id;
	uint8_t crc = payload_size ^ message_id;
	const uint8_t *bytes = static_cast<const uint8_t *>(payload);

	for (size_t i = 0; i < payload_size; ++i) {
		packet[MSP_FRAME_START_SIZE + i] = bytes[i];
		crc ^= bytes[i];
	}

	packet[MSP_FRAME_START_SIZE + payload_size] = crc;
	_tx_end += packet_size;
	return true;
}

int MspV1::Flush()
{
	if (pending_bytes() == 0) {
		return 0;
	}

	const ssize_t written = _io.write(_fd, _tx_buffer + _tx_begin, pending_bytes(), _io.context);

	if (written < 0) {
		return errno == EINTR ? -EAGAIN : -(errno != 0 ? errno : EIO);
	}

	if (written == 0 || static_cast<size_t>(written) > pending_bytes()) {
		return -EIO;
	}

	_tx_begin += written;

	if (_tx_begin == _tx_end) {
		_tx_begin = 0;
		_tx_end = 0;
		return 0;
	}

	return -EAGAIN;
}

int MspV1::Receive(uint8_t *payload, size_t capacity, uint8_t *message_id)
{
	if (message_id == nullptr || (capacity != 0 && payload == nullptr)) {
		return -EINVAL;
	}

	for (size_t processed = 0; processed < RX_BYTE_BUDGET; ++processed) {
		if (_rx_begin == _rx_end) {
			const ssize_t count = _io.read(_fd, _rx_buffer, sizeof(_rx_buffer), _io.context);

			if (count < 0) {
				return errno == EINTR ? -EAGAIN : -(errno != 0 ? errno : EIO);
			}

			if (count == 0 || static_cast<size_t>(count) > sizeof(_rx_buffer)) {
				return -EIO;
			}

			_rx_begin = 0;
			_rx_end = count;
		}

		const uint8_t byte = _rx_buffer[_rx_begin++];

		switch (_rx_state) {
		case RxState::Start:
			if (byte == '$') {
				_rx_state = RxState::Magic;
			}

			break;

		case RxState::Magic:
			_rx_state = byte == 'M' ? RxState::Direction : (byte == '$' ? RxState::Magic : RxState::Start);
			break;

		case RxState::Direction:
			_rx_state = byte == '<' ? RxState::Length : (byte == '$' ? RxState::Magic : RxState::Start);
			break;

		case RxState::Length:
			_rx_size = byte;
			_rx_crc = byte;
			_rx_position = 0;
			_rx_state = RxState::Command;
			break;

		case RxState::Command:
			_rx_id = byte;
			_rx_crc ^= byte;
			_rx_state = _rx_size == 0 ? RxState::Checksum : RxState::Payload;
			break;

		case RxState::Payload:
			_rx_payload[_rx_position++] = byte;
			_rx_crc ^= byte;

			if (_rx_position == _rx_size) {
				_rx_state = RxState::Checksum;
			}

			break;

		case RxState::Checksum:
			_rx_state = RxState::Start;

			if (byte != _rx_crc) {
				return -EBADMSG;
			}

			if (_rx_size > capacity) {
				return -EMSGSIZE;
			}

			if (_rx_size != 0) {
				memcpy(payload, _rx_payload, _rx_size);
			}

			*message_id = _rx_id;
			return _rx_size;
		}
	}

	return -EAGAIN;
}
