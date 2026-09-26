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

#include <drivers/drv_hrt.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>

/** Translate QGC's repeated start commands into one attempt and truthful progress. */
class MavlinkAutotune
{
public:
	struct Response {
		uint8_t result;
		uint8_t progress{0};
		bool publish_command{false};
	};

	Response request(hrt_abstime now, const vehicle_status_s *vehicle_status,
			 const autotune_attitude_control_status_s *status, bool module_supported);

private:
	enum class Phase { Idle, Waiting, Running, Finished };

	Response fail();
	Response progress(const autotune_attitude_control_status_s &status);
	static bool fresh(const autotune_attitude_control_status_s *status, hrt_abstime now);
	static bool terminal(uint8_t state);

	Phase _phase{Phase::Idle};
	hrt_abstime _request_time{0};
	hrt_abstime _last_request_time{0};
	hrt_abstime _attempt_start{0};
	hrt_abstime _last_status_time{0};
	uint8_t _vehicle_type{vehicle_status_s::VEHICLE_TYPE_UNSPECIFIED};
	uint8_t _progress{0};
	bool _terminal_received{false};
};
