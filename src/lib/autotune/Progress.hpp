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

#include <parameters/param.h>
#include <pthread.h>
#include <uORB/uORB.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class AutotuneModuleTest;

namespace autotune
{

/** Process-lifetime, paced text output, independent of either tuner's lifetime. */
class Progress : public px4::ScheduledWorkItem
{
private:
	friend class Session;
	friend class ::AutotuneModuleTest;

	struct Record {
		const char *text; // static string literal, never module-owned storage
		float value;
		param_t parameter;
		uint8_t vehicle_type;
		int8_t axis; // -1 for an attempt-wide message
		bool parameter_write;
		bool warning;
	};

	Progress();
	static Progress &instance(); // caller holds _mutex during construction
	static void enqueue(const Record &record);
	void Run() override;
	void update(hrt_abstime now);

	// A complete 15-parameter MC write followed immediately by rollback fits,
	// with space for stage messages and a subsequent attempt. No heap per entry.
	static constexpr unsigned kCapacity = 64;
	static constexpr hrt_abstime kInterval = 100000; // 10 messages/s; STATUSTEXT runs at 20 Hz
	static pthread_mutex_t _mutex;
	Record _records[kCapacity] {};
	unsigned _head{0};
	unsigned _count{0};
	unsigned _dropped{0};
	hrt_abstime _last_publish{0};
	orb_advert_t _mavlink_log_pub{nullptr};
};

} // namespace autotune
