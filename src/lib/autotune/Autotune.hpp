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
#include <uORB/topics/vehicle_status.h>

namespace autotune
{

// Commander publishes vehicle_status at least every 500 ms. Allow scheduling jitter,
// but never start or continue an experiment on an unknown or stale flight regime.
inline bool eligible(const vehicle_status_s &status, uint8_t vehicle_type, hrt_abstime now)
{
	using namespace time_literals;
	return status.timestamp != 0 && now >= status.timestamp && now - status.timestamp <= 2_s
	       && status.vehicle_type == vehicle_type && !status.in_transition_mode;
}

/**
 * Exclusive writer of the shared excitation/status channel.
 *
 * The single uORB advertisement belongs to this library for the process lifetime,
 * not to either module. Destroying an idle module therefore cannot unadvertise a
 * live experiment. Acquisition, publication and release are serialized; a former
 * owner cannot overwrite its successor's status, including with a terminal sample.
 */
class Session
{
public:
	explicit Session(uint8_t vehicle_type) : _vehicle_type(vehicle_type) {}
	~Session();

	Session(const Session &) = delete;
	Session &operator=(const Session &) = delete;

	bool acquire(hrt_abstime now, hrt_abstime command_timestamp = 0);
	bool ownsChannel() const;
	bool publish(autotune_attitude_control_status_s status);
	void release();

private:
	const uint8_t _vehicle_type;
	hrt_abstime _timestamp_start{0};
};

} // namespace autotune
