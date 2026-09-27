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

#include "Progress.hpp"
#include "Autotune.hpp"

#include <lib/systemlib/mavlink_log.h>
#include <stdio.h>
#include <uORB/topics/mavlink_log.h>

namespace autotune
{

pthread_mutex_t Progress::_mutex = PTHREAD_MUTEX_INITIALIZER;

Progress::Progress() : ScheduledWorkItem("autotune_progress", px4::wq_configurations::lp_default) {}

Progress &Progress::instance()
{
	// Creation is serialized even on targets with non-thread-safe local statics.
	static Progress progress;
	return progress;
}

void Progress::enqueue(const Record &record)
{
	pthread_mutex_lock(&_mutex);
	Progress &progress = instance();

	if (progress._count < kCapacity) {
		progress._records[(progress._head + progress._count) % kCapacity] = record;

		if (progress._count++ == 0) {
			progress.ScheduleNow();
		}

	} else {
		// Preserve ordered audit records already queued; report any loss explicitly.
		++progress._dropped;
	}

	pthread_mutex_unlock(&_mutex);
}

void Progress::Run()
{
	update(hrt_absolute_time());
}

void Progress::update(hrt_abstime now)
{
	pthread_mutex_lock(&_mutex);

	if (_count == 0 && _dropped == 0) {
		pthread_mutex_unlock(&_mutex);
		return;
	}

	if (_last_publish != 0 && now - _last_publish < kInterval) {
		ScheduleDelayed(kInterval - (now - _last_publish));
		pthread_mutex_unlock(&_mutex);
		return;
	}

	Record record{};
	unsigned dropped = 0;

	if (_count > 0) {
		record = _records[_head];
		_head = (_head + 1) % kCapacity;
		--_count;

	} else {
		dropped = _dropped;
		_dropped = 0;
	}

	_last_publish = now;

	if (_count > 0 || _dropped > 0) {
		ScheduleDelayed(kInterval);
	}

	pthread_mutex_unlock(&_mutex);

	char text[sizeof(mavlink_log_s::text)] {};

	if (dropped > 0) {
		snprintf(text, sizeof(text), "AutoTune: progress queue overflow (%u lost)", dropped);
		record.warning = true;

	} else {
		const char *mode = record.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING ? "FW" : "MC";

		if (record.parameter_write) {
			const char *name = param_name(record.parameter);
			snprintf(text, sizeof(text), "AutoTune %s: %s %s=%.9g", mode, record.text,
				 name ? name : "unknown", static_cast<double>(record.value));

		} else if (record.axis >= 0 && record.axis < 3) {
			static constexpr const char *axes[] = {"roll", "pitch", "yaw"};
			snprintf(text, sizeof(text), "AutoTune %s: %s %s", mode, axes[record.axis], record.text);

		} else {
			snprintf(text, sizeof(text), "AutoTune %s: %s", mode, record.text);
		}
	}

	// Both MAVLink STATUSTEXT and PX4 log_message (ULog) receive the paced output.
	if (record.warning) {
		mavlink_log_warning(&_mavlink_log_pub, "%s", text);

	} else {
		mavlink_log_info(&_mavlink_log_pub, "%s", text);
	}
}

void Session::message(const char *text, bool warning)
{
	Progress::enqueue({text, 0.f, PARAM_INVALID, _vehicle_type, -1, false, warning});
}

void Session::axis(int index, const char *text, bool warning)
{
	Progress::enqueue({text, 0.f, PARAM_INVALID, _vehicle_type, static_cast<int8_t>(index), false, warning});
}

void Session::writeParameter(param_t parameter, float value, bool notify, bool restoring)
{
	const bool success = (notify ? param_set(parameter, &value) : param_set_no_notification(parameter, &value)) == PX4_OK;
	const char *action = success ? (restoring ? "restored" : "set") : (restoring ? "RESTORE FAILED" : "WRITE FAILED");
	// Capture the value at the write, not when this record is eventually emitted.
	Progress::enqueue({action, value, parameter, _vehicle_type, -1, true, !success});
}

} // namespace autotune
