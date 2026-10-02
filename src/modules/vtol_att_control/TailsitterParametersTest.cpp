/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <hrt_work.h>
#include "vtol_att_control_main.h"

// Run in a separate executable: another handoff test must not mark these parameters used first.
TEST(TailsitterParameters, HandoffSlewIsDiscoverableAtStartup)
{
	hrt_init();
	hrt_work_queue_init();
	px4_log_initialize();
	param_control_autosave(false);
	param_reset_all();
	const int32_t tailsitter_type = 0;
	ASSERT_EQ(param_set_no_notification(param_find("VT_TYPE"), &tailsitter_type), 0);

	const param_t forward = param_find_no_notification("VT_TS_THR_SLEW");
	const param_t back = param_find_no_notification("VT_TS_B_THR_SLEW");
	ASSERT_NE(forward, PARAM_INVALID);
	ASSERT_NE(back, PARAM_INVALID);
	ASSERT_FALSE(param_used(forward));
	ASSERT_FALSE(param_used(back));

	VtolAttitudeControl controller;

	// MAVLink parameter-list downloads enumerate only used parameters.
	for (const param_t handle : {forward, back}) {
		EXPECT_TRUE(param_used(handle));
		const int index = param_get_used_index(handle);
		ASSERT_GE(index, 0);
		EXPECT_EQ(param_for_used_index(index), handle);
	}

	float value = 0.f;
	ASSERT_EQ(param_get(forward, &value), 0);
	EXPECT_FLOAT_EQ(value, .2f);
	ASSERT_EQ(param_get(back, &value), 0);
	EXPECT_FLOAT_EQ(value, .5f);
}
