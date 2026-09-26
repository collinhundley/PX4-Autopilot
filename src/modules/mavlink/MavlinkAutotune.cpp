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

#include <lib/autotune/Autotune.hpp>

using namespace time_literals;

namespace
{
// Modules publish active status every 100 ms. These limits allow scheduling jitter
// and first startup, but never synthesize initialization indefinitely.
constexpr hrt_abstime STATUS_TIMEOUT = 1_s;
constexpr hrt_abstime START_TIMEOUT = 3_s;
constexpr hrt_abstime RETRY_GAP = 3_s;
using Status = autotune_attitude_control_status_s;
using Ack = vehicle_command_ack_s;
}

bool MavlinkAutotune::fresh(const Status *status, hrt_abstime now)
{
	return status && status->timestamp != 0 && status->timestamp <= now
	       && now - status->timestamp <= STATUS_TIMEOUT && status->timestamp_start != 0
	       && status->timestamp_start <= status->timestamp;
}

bool MavlinkAutotune::terminal(uint8_t state)
{
	return state == Status::STATE_COMPLETE || state == Status::STATE_FAIL;
}

MavlinkAutotune::Response MavlinkAutotune::fail()
{
	_phase = Phase::Finished;
	return {Ack::VEHICLE_CMD_RESULT_FAILED};
}

MavlinkAutotune::Response MavlinkAutotune::request(hrt_abstime now, const vehicle_status_s *vehicle_status,
		const Status *status, bool module_supported)
{
	const bool retry_gap = _last_request_time != 0 && now - _last_request_time > RETRY_GAP;
	_last_request_time = now;
	const bool fresh_status = fresh(status, now);

	// The protocol has no attempt id: QGC sends identical start commands at 1 Hz.
	// After a missing/stopped module fails, require a pause before retrying so
	// continued polls cannot keep starting new three-second handshake windows.
	// An observed terminal -> IDLE transition also permits a subsequent tune.
	if (_phase == Phase::Finished
	    && (retry_gap || (_terminal_received && fresh_status && status->state == Status::STATE_IDLE
			      && status->vehicle_type == _vehicle_type && status->timestamp_start == _attempt_start))) {
		_phase = Phase::Idle;
		_terminal_received = false;
	}

	if (!vehicle_status || !autotune::eligible(*vehicle_status, vehicle_status->vehicle_type, now)) {
		if (_phase == Phase::Waiting || _phase == Phase::Running) {
			return fail();
		}

		return {Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED};
	}

	if (_phase == Phase::Finished) {
		// Never report the previous attempt's success as the outcome of a new click
		// during the module's terminal-state cooldown.
		return {_terminal_received ? Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED : Ack::VEHICLE_CMD_RESULT_FAILED};
	}

	if (_phase == Phase::Idle) {
		if (!module_supported) {
			return {Ack::VEHICLE_CMD_RESULT_UNSUPPORTED};
		}

		_vehicle_type = vehicle_status->vehicle_type;
		_progress = 0;
		_attempt_start = 0;

		if (fresh_status && status->state != Status::STATE_IDLE) {
			if (status->vehicle_type == _vehicle_type && !terminal(status->state)) {
				// Attach to an experiment already started by a mission or the FW AUX switch.
				_attempt_start = status->timestamp_start;
				_phase = Phase::Running;
				return progress(*status);
			}

			if (!terminal(status->state) || status->vehicle_type == _vehicle_type) {
				// Another owner is still active, or this module is still in cooldown.
				return {Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED};
			}
		}

		_request_time = now;
		_phase = Phase::Waiting;
		return {Ack::VEHICLE_CMD_RESULT_IN_PROGRESS, 0, true};
	}

	if (vehicle_status->vehicle_type != _vehicle_type) {
		return fail();
	}

	if (_phase == Phase::Waiting) {
		// A mission, AUX edge or another receiver may have acquired the channel just
		// before our request, without its first publication being visible yet. A
		// subsequent active heartbeat proves there is an experiment to follow.
		// Terminal outcomes still need to belong to an attempt begun after our request.
		const bool active_heartbeat = fresh_status && status->timestamp >= _request_time
					      && status->state != Status::STATE_IDLE && !terminal(status->state);

		if (fresh_status && status->vehicle_type == _vehicle_type
		    && (status->timestamp_start >= _request_time || active_heartbeat)) {
			_attempt_start = status->timestamp_start;
			_phase = Phase::Running;
			return progress(*status);
		}

		if (now - _request_time >= START_TIMEOUT) {
			return fail();
		}

		return {Ack::VEHICLE_CMD_RESULT_IN_PROGRESS};
	}

	if (fresh_status && status->vehicle_type == _vehicle_type && status->timestamp_start == _attempt_start) {
		return progress(*status);
	}

	if (now - _last_status_time >= STATUS_TIMEOUT) {
		return fail();
	}

	return {Ack::VEHICLE_CMD_RESULT_IN_PROGRESS, _progress};
}

MavlinkAutotune::Response MavlinkAutotune::progress(const Status &status)
{
	_last_status_time = status.timestamp;

	switch (status.state) {
	case Status::STATE_INIT:
		_progress = 0;
		break;

	case Status::STATE_ROLL_AMPLITUDE_DETECTION:
	case Status::STATE_ROLL:
	case Status::STATE_ROLL_PAUSE:
		_progress = 20;
		break;

	case Status::STATE_PITCH_AMPLITUDE_DETECTION:
	case Status::STATE_PITCH:
	case Status::STATE_PITCH_PAUSE:
		_progress = 40;
		break;

	case Status::STATE_YAW_AMPLITUDE_DETECTION:
	case Status::STATE_YAW:
	case Status::STATE_YAW_PAUSE:
		_progress = 60;
		break;

	case Status::STATE_VERIFICATION:
		_progress = 80;
		break;

	case Status::STATE_APPLY:
		_progress = 85;
		break;

	case Status::STATE_TEST:
		_progress = 90;
		break;

	case Status::STATE_WAIT_FOR_DISARM:
		_progress = 95;
		break;

	case Status::STATE_COMPLETE:
		_terminal_received = true;
		_phase = Phase::Finished;
		return {Ack::VEHICLE_CMD_RESULT_ACCEPTED, 100};

	case Status::STATE_FAIL:
		_terminal_received = true;
		return fail();

	default:
		// IDLE without an observed terminal state is not evidence of success.
		return fail();
	}

	return {Ack::VEHICLE_CMD_RESULT_IN_PROGRESS, _progress};
}
