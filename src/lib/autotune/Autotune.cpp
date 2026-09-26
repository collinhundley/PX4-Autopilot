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

#include "Autotune.hpp"

#include <pthread.h>
#include <px4_platform_common/defines.h>
#include <uORB/uORB.h>

namespace autotune
{
namespace
{
pthread_mutex_t channel_mutex = PTHREAD_MUTEX_INITIALIZER;
const Session *owner{nullptr};
hrt_abstime last_command_timestamp{0};
// Intentionally retained across module stops/restarts. uORB owns the backing node
// for the process lifetime. No module-local publication may advertise this topic.
orb_advert_t status_publication{nullptr};
}

Session::~Session()
{
	// Modules publish their final state and restore trial gains before destruction.
	// This fallback also prevents a dangling owner after a failed initialization.
	if (ownsChannel()) {
		autotune_attitude_control_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.state = autotune_attitude_control_status_s::STATE_FAIL;
		publish(status);
		release();
	}
}

bool Session::acquire(hrt_abstime now, hrt_abstime command_timestamp)
{
	pthread_mutex_lock(&channel_mutex);
	// Both modules subscribe to the command queue. A delayed consumer must not
	// start the same command again after the first owner has aborted/released.
	const bool acquired = owner == nullptr
			      && (command_timestamp == 0 || command_timestamp > last_command_timestamp);

	if (acquired) {
		owner = this;
		_timestamp_start = now;

		if (command_timestamp != 0) {
			last_command_timestamp = command_timestamp;
		}
	}

	pthread_mutex_unlock(&channel_mutex);
	return acquired;
}

bool Session::ownsChannel() const
{
	pthread_mutex_lock(&channel_mutex);
	const bool owns = owner == this;
	pthread_mutex_unlock(&channel_mutex);
	return owns;
}

bool Session::publish(autotune_attitude_control_status_s status)
{
	pthread_mutex_lock(&channel_mutex);
	bool published = false;

	if (owner == this) {
		status.timestamp = hrt_absolute_time();
		status.timestamp_start = _timestamp_start;
		status.vehicle_type = _vehicle_type;

		if (status_publication == nullptr) {
			status_publication = orb_advertise(ORB_ID(autotune_attitude_control_status), &status);
			published = status_publication != nullptr;

		} else {
			published = orb_publish(ORB_ID(autotune_attitude_control_status), status_publication, &status) == PX4_OK;
		}
	}

	pthread_mutex_unlock(&channel_mutex);
	return published;
}

void Session::release()
{
	pthread_mutex_lock(&channel_mutex);

	if (owner == this) {
		owner = nullptr;
	}

	pthread_mutex_unlock(&channel_mutex);
}

} // namespace autotune
