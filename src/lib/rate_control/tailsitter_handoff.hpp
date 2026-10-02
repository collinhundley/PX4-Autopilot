/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

#include <matrix/matrix/math.hpp>
#include "rate_control.hpp"
#include <mathlib/mathlib.h>

namespace tailsitter_handoff
{
inline matrix::Vector3f toFW(const matrix::Vector3f &v) { return {-v(2), v(1), v(0)}; }
inline matrix::Vector3f toMC(const matrix::Vector3f &v) { return {v(2), v(1), -v(0)}; }

// Remove only the initial mismatch, leaving subsequent feedback and pilot input unfiltered.
// 0.5 s gives the attitude/rate loops time to acquire the FW target without adding feedback lag.
static constexpr float kTorqueBlendTime = 0.5f;
// Allows several normal publication intervals and scheduler jitter without accepting stale output.
static constexpr uint64_t kOutputMaxAge = 100000; // 100 ms
static constexpr uint64_t kOutputTimeout = 1000000; // 1 s before returning to MC
inline float remaining(float elapsed)
{
	const float x = math::constrain(elapsed / kTorqueBlendTime, 0.f, 1.f);
	return 1.f - x * x * (3.f - 2.f * x);
}

// One-shot collective slew. Finish only when both the live demand and its source slew
// can be followed, then pass all later commands unchanged until the next handoff.
class ThrottleSlew
{
public:
	void reset(float output, float rate)
	{
		_output = math::constrain(output, 0.f, 1.f);
		_rate = PX4_ISFINITE(rate) ? math::max(rate, 0.f) : 0.f;
		_previous_demand = NAN;
		_previous_timestamp = 0;
		_demand_slew_compatible = false;
		_active = true;
	}

	float update(float demand, float dt, uint64_t timestamp, bool underspeed = false)
	{
		// Never interpret an absent publication as a motor-stop command.
		if (!PX4_ISFINITE(demand)) { return NAN; }

		demand = math::constrain(demand, 0.f, 1.f);

		if (!_active) { return demand; }

		// Faster rises preserve lift/airspeed margin. No special pilot-stick bypass.
		const float rise = 2.f * _rate;

		if (timestamp > _previous_timestamp) {
			const float source_dt = (timestamp - _previous_timestamp) * 1e-6f;
			const float delta = demand - _previous_demand;
			_demand_slew_compatible = PX4_ISFINITE(_previous_demand)
						  && delta >= -_rate * source_dt - FLT_EPSILON
						  && (delta <= rise * source_dt + FLT_EPSILON || underspeed);
			_previous_timestamp = timestamp;
			_previous_demand = demand;
		}

		// Pending acknowledgement and the first accepted output retain exact MC collective.
		if (dt <= 0.f) { return _output; }

		if (_rate <= FLT_EPSILON) {
			_output = demand;
			_active = false;

		} else {
			const float delta = demand - _output;
			_output += math::constrain(delta, -_rate * dt, underspeed ? math::max(delta, 0.f) : rise * dt);

			if (fabsf(demand - _output) <= FLT_EPSILON && _demand_slew_compatible) {
				_output = demand;
				_active = false;
			}
		}

		return _output;
	}

	bool active() const { return _active; }
	void cancel() { _active = false; }

private:
	float _output{0.f};
	float _rate{0.f};
	float _previous_demand{NAN};
	uint64_t _previous_timestamp{0};
	bool _demand_slew_compatible{false};
	bool _active{false};
};

// Motor scaling is in MC coordinates. A disabled axis is deliberately not inverted.
inline matrix::Vector3f motorToFW(const matrix::Vector3f &motor, const matrix::Vector3f &scale)
{
	matrix::Vector3f result;

	for (int i = 0; i < 3; ++i) {
		result(i) = scale(i) > FLT_EPSILON ? motor(i) / scale(i) : 0.f;
	}

	return toFW(result);
}

// Return only the bias that cannot be represented by the bounded FW integrator.
inline matrix::Vector3f initializeBias(RateControl &controller, const matrix::Vector3f &mc_bias,
				       const matrix::Vector3f &motor_scale, const matrix::Vector3f &trim,
				       const matrix::Vector3f &gain, float roll_to_yaw)
{
	matrix::Vector3f bias = matrix::constrain(motorToFW(mc_bias, motor_scale), -1.f, 1.f);
	const matrix::Vector3f scale(motor_scale(2), motor_scale(1), motor_scale(0));
	// A surface-only roll axis retains its scheduled trim.
	bias(2) -= roll_to_yaw * (scale(0) > FLT_EPSILON ? bias(0) : trim(0));
	matrix::Vector3f integral;

	for (int i = 0; i < 3; ++i) {
		if (scale(i) > FLT_EPSILON && gain(i) > FLT_EPSILON) { integral(i) = (bias(i) - trim(i)) / gain(i); }
	}

	controller.setIntegral(integral);
	matrix::Vector3f residual = bias - trim - gain.emult(controller.getIntegral());

	for (int i = 0; i < 3; ++i) {
		if (scale(i) <= FLT_EPSILON) { residual(i) = 0.f; }
	}

	return residual;
}

inline bool ready(uint64_t request, uint64_t ack, uint64_t sample, uint64_t torque_sample, uint64_t thrust_sample)
{
	return request != 0 && ack == request && sample > request && torque_sample >= sample && thrust_sample >= sample;
}
}
