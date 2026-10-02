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

// X-Plane-only test fixture. Add bounded body moments without overriding aircraft dynamics.
// Disabled on load; the caller must refresh a finite-duration request for every flight.
#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include <algorithm>
#include <cmath>
#include <cstring>

static float pitch_nm{}, roll_nm{}, yaw_nm{}, remaining{};
static XPLMDataRef pitch_ref{}, roll_ref{}, yaw_ref{}, aircraft_ref{}, paused_ref{};
static XPLMDataRef custom[4] {};
static XPLMFlightLoopID loop{};

static float get(void *value) { return *static_cast<float *>(value); }
static void set(void *value, float input)
{
	if (std::isfinite(input)) {
		*static_cast<float *>(value) = value == &remaining ? std::clamp(input, 0.f, 180.f) : std::clamp(input, -2.f, 2.f);
	}
}

static float before(float dt, float, int, void *)
{
	char path[1024] {};
	XPLMGetDatab(aircraft_ref, path, 0, sizeof(path) - 1);

	if (std::strcmp(path, "Aircraft/PX4/Dragonfly/Dragonfly.acf") != 0 || XPLMGetDatai(paused_ref)) { return -1.f; }

	if (remaining > 0.f) {
		// SDK force accumulators are reset every frame. Preserve all other plugin contributions.
		XPLMSetDataf(pitch_ref, XPLMGetDataf(pitch_ref) + pitch_nm);
		XPLMSetDataf(roll_ref, XPLMGetDataf(roll_ref) + roll_nm);
		XPLMSetDataf(yaw_ref, XPLMGetDataf(yaw_ref) + yaw_nm);
		remaining = std::max(0.f, remaining - std::max(dt, 0.f));
	}

	return -1.f;
}

PLUGIN_API int XPluginStart(char *name, char *signature, char *description)
{
	std::strcpy(name, "Dragonfly handoff test moments");
	std::strcpy(signature, "uav.dragonfly.handoff_test_moments");
	std::strcpy(description, "Temporary signed body-moment disturbance fixture; disabled by default.");
	pitch_ref = XPLMFindDataRef("sim/flightmodel/forces/M_plug_acf");
	roll_ref = XPLMFindDataRef("sim/flightmodel/forces/L_plug_acf");
	yaw_ref = XPLMFindDataRef("sim/flightmodel/forces/N_plug_acf");
	aircraft_ref = XPLMFindDataRef("sim/aircraft/view/acf_relative_path");
	paused_ref = XPLMFindDataRef("sim/time/paused");

	if (!pitch_ref || !roll_ref || !yaw_ref || !aircraft_ref || !paused_ref) { return 0; }

	const char *names[] = {"dragonfly/handoff_test/pitch_nm", "dragonfly/handoff_test/roll_nm",
			       "dragonfly/handoff_test/yaw_nm", "dragonfly/handoff_test/seconds_remaining"
			      };
	float *values[] = {&pitch_nm, &roll_nm, &yaw_nm, &remaining};

	for (int i = 0; i < 4; ++i) {
		custom[i] = XPLMRegisterDataAccessor(names[i], xplmType_Float, 1, nullptr, nullptr, get, set,
						     nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, values[i], values[i]);
	}

	XPLMCreateFlightLoop_t config{sizeof(config), xplm_FlightLoop_Phase_BeforeFlightModel, before, nullptr};
	loop = XPLMCreateFlightLoop(&config);
	XPLMScheduleFlightLoop(loop, -1.f, 1);
	return 1;
}

PLUGIN_API void XPluginStop()
{
	if (loop) { XPLMDestroyFlightLoop(loop); }

	for (auto ref : custom) { if (ref) { XPLMUnregisterDataAccessor(ref); } }
}
PLUGIN_API int XPluginEnable() { remaining = 0.f; return 1; }
PLUGIN_API void XPluginDisable() { remaining = 0.f; }
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int, void *) {}
