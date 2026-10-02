/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
#include "tailsitter_handoff.hpp"

using matrix::Vector3f;
using namespace tailsitter_handoff;

TEST(TailsitterHandoff, ThreeAxisBiasWithTrimGainAndDifferentialScaling)
{
	for (float gain_scale : {0.25f, 1.f, 4.f}) {
		for (float yaw_ff : {-0.3f, 0.f, 0.5f}) {
			RateControl controller;
			const Vector3f limits(.08f, .1f, .06f);
			controller.setIntegratorLimit(limits);
			const Vector3f mc_bias(.07f, -.13f, -.11f); // includes crosswind yaw/roll trim
			const Vector3f scale(.7f, .8f, .4f);
			const Vector3f trim(.03f, -.02f, .01f);
			const Vector3f gain = Vector3f(.6f, .8f, 1.f) * gain_scale;
			const Vector3f residual = initializeBias(controller, mc_bias, scale, trim, gain, yaw_ff);
			Vector3f fw = gain.emult(controller.getIntegral()) + residual + trim;
			fw(2) += yaw_ff * fw(0);
			const Vector3f motor = toMC(fw).emult(scale);

			for (int i = 0; i < 3; ++i) {
				EXPECT_NEAR(motor(i), mc_bias(i), 1e-6f);
				EXPECT_LE(fabsf(controller.getIntegral()(i)), limits(i));
			}
		}
	}
}

TEST(TailsitterHandoff, DisabledAxesAndZeroScaleAreNotInverted)
{
	RateControl controller;
	controller.setIntegratorLimit(Vector3f(.2f, .2f, .2f));
	const Vector3f trim(.05f, -.03f, .01f);
	const Vector3f residual = initializeBias(controller, Vector3f(.1f, -.12f, .3f),
				  Vector3f(0.f, .5f, 0.f), trim, Vector3f(1.f, 1.f, 1.f), .5f);
	EXPECT_FLOAT_EQ(residual(0), 0.f);
	EXPECT_FLOAT_EQ(residual(2), 0.f);
	EXPECT_FLOAT_EQ(controller.getIntegral()(0), 0.f);
	EXPECT_FLOAT_EQ(controller.getIntegral()(2), 0.f);
	EXPECT_NEAR(.5f * (controller.getIntegral()(1) + trim(1) + residual(1)), -.12f, 1e-6f);
}

TEST(TailsitterHandoff, PDAndFeedforwardMismatchDoesNotChangeFirstCommand)
{
	RateControl controller;
	controller.setPidGains(Vector3f(.1f, .15f, .2f), Vector3f(.1f, .2f, .3f), Vector3f(.01f, .02f, .03f));
	controller.setFeedForwardGain(Vector3f(.5f, -.1f, .2f));
	controller.setIntegratorLimit(Vector3f(.1f, .1f, .1f));
	const Vector3f scale(.8f, .6f, .9f), gain(.5f, .7f, .8f), trim(.02f, -.03f, .04f);
	const Vector3f motor(.15f, -.2f, .1f), bias(.06f, -.1f, .08f);
	const Vector3f residual = initializeBias(controller, bias, scale, trim, gain, 0.f);
	const Vector3f current = gain.emult(controller.update(Vector3f(.1f, -.2f, .3f),
					    Vector3f(.3f, -.5f, .2f), Vector3f(1.f, -2.f, 3.f), .004f, true)) + trim + residual;
	const Vector3f offset = motorToFW(motor, scale) - current;

	for (float elapsed : {0.f, .1f, .25f, .5f, 1.f}) {
		const Vector3f output = toMC(current + remaining(elapsed) * offset).emult(scale);
		EXPECT_TRUE(output.isAllFinite());

		if (elapsed == 0.f) { EXPECT_LT((output - motor).norm(), 1e-6f); }

		if (elapsed >= .5f) { EXPECT_LT((output - toMC(current).emult(scale)).norm(), 1e-6f); }
	}
}

TEST(TailsitterHandoff, ArbitraryStickCurvesAndAsymmetricConvergence)
{
	for (float outgoing : {.08f, .2336f, .5f, .95f}) {
		for (float stick : {0.f, .05f, .25f, .5f, .9f, 1.f}) {
			ThrottleSlew slew;
			slew.reset(outgoing, .2f);
			float previous = slew.update(stick, 0.f, 1);
			EXPECT_FLOAT_EQ(previous, outgoing);

			for (int sample = 1; sample < 1500; ++sample) {
				const float next = slew.update(stick, .004f, 1 + sample * 4000);
				EXPECT_LE(fabsf(next - previous), (stick > outgoing ? .0016f : .0008f) + 1e-6f);
				EXPECT_GE(next, 0.f); EXPECT_LE(next, 1.f);
				previous = next;
			}

			EXPECT_NEAR(previous, stick, 1e-5f);
			EXPECT_FALSE(slew.active());
			// Completed means normal FW: subsequent demand jumps are not constrained.
			EXPECT_FLOAT_EQ(slew.update(1.f - stick, .004f, 6000001), 1.f - stick);
		}
	}
}

TEST(TailsitterHandoff, LiveDemandChangesAndRepeatedSourceSamplesDoNotEndSlewEarly)
{
	ThrottleSlew slew;
	slew.reset(.4f, .2f);
	slew.update(.8f, 0.f, 100000);
	// Demand crosses the output while still changing faster than the allowed fall.
	EXPECT_NEAR(slew.update(.4f, .02f, 120000), .4f, 1e-6f);
	EXPECT_TRUE(slew.active());
	// Republishing this same source must not be mistaken for settled FW demand.
	slew.update(.4f, .01f, 120000);
	EXPECT_TRUE(slew.active());
	EXPECT_NEAR(slew.update(.1f, .02f, 140000), .396f, 1e-6f);
	EXPECT_TRUE(slew.active());
	// No special rapid manual-throttle bypass.
	EXPECT_NEAR(slew.update(1.f, .02f, 160000), .404f, 1e-6f);
	EXPECT_TRUE(slew.active());
}

TEST(TailsitterHandoff, BackSymmetricSlewAcquiresChangingDemand)
{
	ThrottleSlew slew;
	slew.reset(.09f, .5f, kBackThrottleRiseScale);
	EXPECT_FLOAT_EQ(slew.update(.32f, 0.f, 1), .09f);
	EXPECT_NEAR(slew.update(.32f, .1f, 100001), .14f, 1e-6f);
	EXPECT_NEAR(slew.update(.32f, .1f, 200001), .19f, 1e-6f);
	EXPECT_NEAR(slew.update(.08f, .1f, 300001), .14f, 1e-6f);
	EXPECT_TRUE(slew.active());
	EXPECT_NEAR(slew.update(.08f, .1f, 400001), .09f, 1e-6f);
	EXPECT_TRUE(slew.active());
	EXPECT_NEAR(slew.update(.08f, .1f, 500001), .08f, 1e-6f);
	EXPECT_FALSE(slew.active());
	EXPECT_FLOAT_EQ(slew.update(.9f, .01f, 510001), .9f);
}

TEST(TailsitterHandoff, DelayedPublicationsAndOldAcknowledgementsNeverBecomeReady)
{
	EXPECT_FALSE(ready(0, 0, 0, 0, 0));
	EXPECT_FALSE(ready(100, 90, 110, 110, 110));
	EXPECT_FALSE(ready(100, 100, 0, 0, 0));
	EXPECT_FALSE(ready(100, 100, 90, 110, 110));
	EXPECT_FALSE(ready(100, 100, 110, 110, 90));
	EXPECT_FALSE(ready(100, 100, 110, 90, 110));
	EXPECT_TRUE(ready(100, 100, 110, 110, 110));
	// Readiness depends on generation/sample, not elapsed time: still works after >50 ms.
	EXPECT_TRUE(ready(100, 100, 200000, 200000, 200000));
	EXPECT_FALSE(ready(300000, 100, 200000, 200000, 200000));
}

TEST(TailsitterHandoff, SaturationStopsIntegrationAndAllowsRecovery)
{
	RateControl controller;
	controller.setPidGains(Vector3f(), Vector3f(1.f, 1.f, 1.f), Vector3f());
	controller.setIntegratorLimit(Vector3f(.1f, .2f, .3f));
	controller.setIntegral(Vector3f(4.f, -4.f, NAN));
	EXPECT_EQ(controller.getIntegral(), Vector3f(.1f, -.2f, 0.f));
	controller.setPositiveSaturationFlag(0, true);
	controller.setNegativeSaturationFlag(1, true);
	controller.update(Vector3f(), Vector3f(1.f, -1.f, 0.f), Vector3f(), .01f, false);
	EXPECT_EQ(controller.getIntegral(), Vector3f(.1f, -.2f, 0.f));
	controller.update(Vector3f(), Vector3f(-1.f, 1.f, 0.f), Vector3f(), .01f, false);
	EXPECT_LT(controller.getIntegral()(0), .1f);
	EXPECT_GT(controller.getIntegral()(1), -.2f);
	// MC positive yaw residual becomes negative FW roll residual.
	EXPECT_EQ(toFW(Vector3f(.1f, -.2f, .3f)), Vector3f(-.3f, -.2f, .1f));
}

TEST(TailsitterHandoff, RepeatedAndAbortedTransitionStateDoesNotAccumulate)
{
	RateControl controller;
	controller.setIntegratorLimit(Vector3f(.1f, .1f, .1f));
	ThrottleSlew throttle;

	for (float sign : {1.f, -1.f, 1.f}) {
		const Vector3f bias = sign * Vector3f(.1f, -.2f, .3f);
		const Vector3f residual = initializeBias(controller, bias, Vector3f(1.f, 1.f, 1.f),
					  Vector3f(), Vector3f(1.f, 1.f, 1.f), 0.f);
		EXPECT_LT((toMC(controller.getIntegral() + residual) - bias).norm(), 1e-6f);
		throttle.reset(.3f, .2f);
		EXPECT_NEAR(throttle.update(.9f, 0.f, 1), .3f, 1e-6f);
		controller.resetIntegral();
		EXPECT_EQ(controller.getIntegral(), Vector3f());
	}
}

TEST(TailsitterHandoff, UnderspeedBypassesOnlyTheAdditionalIncreaseLimit)
{
	ThrottleSlew slew;
	slew.reset(.2f, .2f);
	EXPECT_NEAR(slew.update(.9f, 0.f, 1), .2f, 1e-6f);
	EXPECT_FLOAT_EQ(slew.update(1.f, .02f, 20001, true), 1.f);
}

TEST(TailsitterHandoff, InvalidDemandDoesNotBecomeZeroOrDestroyTheMatch)
{
	ThrottleSlew slew;
	slew.reset(.4f, .2f);
	EXPECT_FALSE(PX4_ISFINITE(slew.update(NAN, .1f, 1)));
	EXPECT_NEAR(slew.update(.8f, 0.f, 2), .4f, 1e-6f);
}

TEST(TailsitterHandoff, DisabledSlewStillMatchesFirstOutput)
{
	ThrottleSlew slew;
	slew.reset(.4f, 0.f);
	EXPECT_FLOAT_EQ(slew.update(.8f, 0.f, 1), .4f);
	EXPECT_FLOAT_EQ(slew.update(.8f, .01f, 10001), .8f);
	EXPECT_FALSE(slew.active());
}
