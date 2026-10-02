/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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

/**
* @file tailsitter.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#ifndef TAILSITTER_H
#define TAILSITTER_H

#include "vtol_type.h"
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/tailsitter_handoff.h>
#include <uORB/topics/tailsitter_handoff_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/control_allocator_status.h>

#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>

// [rad] Pitch threshold required for completing transition to fixed-wing in automatic transitions
static constexpr float PITCH_THRESHOLD_AUTO_TRANSITION_TO_FW = -1.05f; // -60°

// [rad] Pitch threshold required for completing transition to hover in automatic transitions
static constexpr float PITCH_THRESHOLD_AUTO_TRANSITION_TO_MC = -0.26f; // -15°

class Tailsitter : public VtolType
{
	friend class TailsitterHandoffTest;

public:
	Tailsitter(VtolAttitudeControl *_att_controller);
	~Tailsitter() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void update_mc_state() override;
	void update_fw_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;

private:
	uORB::Subscription _handoff_mc_sub{ORB_ID(tailsitter_handoff_mc)};
	uORB::Subscription _handoff_fw_sub{ORB_ID(tailsitter_handoff_fw)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _allocator_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _handoff_ack_sub{ORB_ID(tailsitter_handoff_ack)};
	uORB::Publication<tailsitter_handoff_s> _handoff_pub{ORB_ID(tailsitter_handoff)};
	tailsitter_handoff_s _handoff{};
	matrix::Vector3f _handoff_motor_residual{};
	hrt_abstime _handoff_active_timestamp{0};
	hrt_abstime _last_valid_fw_output{0};
	hrt_abstime _mc_takeover_timestamp{0};
	hrt_abstime _back_transition_end_timestamp{0};
	matrix::Vector3f _back_attitude_offset{};
	uORB::Subscription _handoff_status_sub{ORB_ID(tailsitter_handoff_status)};

	void startHandoff(bool to_mc = false);
	void startBackHandoff();
	void cancelHandoff();
	bool backHandoffSafe();
	bool usesFwThrottleHandoff() const override { return true; }

	enum class vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
		FW_MODE					/**< vtol is in fixed wing mode */
	};

	vtol_mode _vtol_mode{vtol_mode::MC_MODE};			/**< vtol flight mode, defined by enum vtol_mode */

	bool _flag_was_in_trans_mode = false;	// true if mode has just switched to transition

	matrix::Quatf _q_trans_start;
	matrix::Quatf _q_trans_sp;
	matrix::Vector3f _trans_rot_axis;

	inline static const matrix::Quatf _q_fw_to_mc{matrix::Eulerf{0.f, M_PI_2_F, 0.f}};

	void parameters_update() override;

	bool isFrontTransitionCompletedBase() override;
	bool isPitchExceeded() override;
	bool isRollExceeded() override;

	matrix::Eulerf getFixedWingAttitudeEuler() const;

	DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
					(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off
				       )


};
#endif
