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

#include "Srxl2Telemetry.hpp"
#include "Srxl2Transport.hpp"

#include <lib/rc/srxl2/Srxl2.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/parameter_update.h>

class Srxl2Rc : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;
	Srxl2Rc(const char *device, uint32_t uid);
	~Srxl2Rc() override;
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:
	void Run() override;
	void stop();
	void process_packet(const srxl2::Packet &packet, hrt_abstime now);
	void send_reply(hrt_abstime now);
	void check_control_timing();
	void publish(hrt_abstime now);

	static constexpr uint32_t PollIntervalUs = 250;
	static constexpr size_t ReadSize = 128;
	static constexpr unsigned MaxReadsPerRun = 2;
	static constexpr uint64_t TelemetryUpdateUs = 20000;
	static constexpr uint64_t PendingReplyTimeoutUs = 3000;

	char _device[32] {};
	Srxl2Transport _transport;
	// Persistent scratch avoids stacking RX buffers beneath telemetry and uORB calls.
	uint8_t _rx_buffer[ReadSize] {};
	srxl2::Packet _packet {};
	srxl2::Parser _parser;
	srxl2::Endpoint _endpoint;
	srxl2::ControlTiming _control_timing;
	Srxl2Telemetry _telemetry;
	srxl2::ReplyRequest _reply {};
	uint64_t _reply_first_sequence {0};
	uint64_t _reply_end_sequence {0};
	hrt_abstime _reply_created {0};
	hrt_abstime _telemetry_updated {0};
	uint64_t _control_first_sequence {0};
	uint64_t _control_end_sequence {0};
	size_t _control_length {0};
	bool _check_timing {false};
	bool _published {false};
	bool _last_lost {true};
	bool _last_failsafe {false};
	uint32_t _bytes_rx {0};
	uint32_t _invalid_packets {0};
	uint32_t _unqualified_packets {0};
	uint32_t _tx_sent {0};
	uint32_t _tx_skipped {0};
	uint32_t _io_errors {0};

	uORB::PublicationMulti<input_rc_s> _input_rc_pub {ORB_ID(input_rc)};
	uORB::SubscriptionInterval _parameter_update_sub {ORB_ID(parameter_update), 1000000};
	perf_counter_t _cycle_perf {perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _publish_perf {perf_alloc(PC_INTERVAL, MODULE_NAME ": RC publication")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::RC_SRXL2_TEL_EN>) _param_telemetry_enabled
	)
};
