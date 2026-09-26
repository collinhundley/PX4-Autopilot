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

#include "Srxl2Rc.hpp"

#include <px4_platform_common/board_common.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <cerrno>
#include <inttypes.h>
#include <string.h>

ModuleBase::Descriptor Srxl2Rc::desc {task_spawn, custom_command, print_usage};

Srxl2Rc::Srxl2Rc(const char *device, uint32_t uid) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device)),
	_endpoint(uid)
{
	strncpy(_device, device, sizeof(_device) - 1);
}

Srxl2Rc::~Srxl2Rc()
{
	_transport.close();
	perf_free(_cycle_perf);
	perf_free(_publish_perf);
}

int Srxl2Rc::task_spawn(int argc, char *argv[])
{
	int index = 1;
	const char *argument = nullptr;
	const char *device = nullptr;
	int option;

	while ((option = px4_getopt(argc, argv, "d:", &index, &argument)) != EOF) {
		if (option != 'd') {
			return print_usage("invalid option");
		}

		device = argument;
	}

	if (!device || strlen(device) >= sizeof(_device)) {
		return print_usage("valid UART device required");
	}

	if (board_rc_conflicting(device)) {
		PX4_ERR("UART conflicts with PX4IO");
		return PX4_ERROR;
	}

	px4_guid_t guid {};
	board_get_px4_guid(guid);
	// FNV-1a folds the board identity into SRXL2's 32-bit device UID.
	uint32_t uid = 2166136261u;

	for (uint8_t byte : guid) {
		uid = (uid ^ byte) * 16777619u;
	}

	auto *instance = new Srxl2Rc(device, uid);

	if (!instance) {
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return PX4_OK;
}

void Srxl2Rc::Run()
{
	if (should_exit()) {
		stop();
		return;
	}

	// NuttX file descriptors belong to a task group. The command that starts
	// us and the work queue have different descriptor tables, so configure,
	// use and close the UART from this work queue only.
	if (!_transport.isOpen()) {
		if (!_transport.open(_device)) {
			PX4_ERR("cannot configure timed half-duplex UART %s (%d)", _device, _transport.lastError());
			stop();
			return;
		}

		ScheduleOnInterval(PollIntervalUs);
	}

	perf_begin(_cycle_perf);
	const hrt_abstime now = hrt_absolute_time();

	if (_endpoint.tick(now)) {
		_parser.reset();
		_reply = {};
		_control_timing.invalidate();
		_check_timing = false;
	}

	for (unsigned iteration = 0; iteration < MaxReadsPerRun; ++iteration) {
		Srxl2Transport::RxChunk chunk {};
		const ssize_t count = _transport.read(_rx_buffer, sizeof(_rx_buffer), chunk);

		if (count < 0) {
			++_io_errors;
			const int error = _transport.lastError();

			if (error != EAGAIN && error != EINTR) {
				PX4_ERR("UART read failed (%d), stopping", error);
				perf_end(_cycle_perf);
				stop();
				return;
			}

			break;
		}

		if (count == 0) {
			break;
		}

		_bytes_rx += count;

		for (ssize_t i = 0; i < count; ++i) {
			_packet = {};

			if (_parser.feed(_rx_buffer[i], chunk.first_sequence + i, _packet)) {
				do {
					process_packet(_packet, now);
				} while (_parser.next(_packet));
			}
		}
	}

	check_control_timing();
	send_reply(hrt_absolute_time());

	const auto state = _endpoint.channel_state(now);

	if (!_published || state.rc_lost != _last_lost || state.rc_failsafe != _last_failsafe) {
		publish(now);
	}

	// Subscriber work and payload encoding are outside the receive/reply path.
	if (now - _telemetry_updated >= TelemetryUpdateUs) {
		_telemetry.update(now);
		_telemetry_updated = now;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s update {};
		_parameter_update_sub.copy(&update);
		updateParams();
	}

	perf_end(_cycle_perf);
}

void Srxl2Rc::stop()
{
	ScheduleClear();
	_transport.close();

	if (_published) {
		_endpoint.reset();
		publish(hrt_absolute_time());
	}

	exit_and_cleanup(desc);
}

void Srxl2Rc::process_packet(const srxl2::Packet &packet, hrt_abstime now)
{
	Srxl2Transport::Timing timing {};

	if (!_transport.packetTiming(packet.first_sequence, packet.last_sequence + 1, timing)
	    || !timing.valid || timing.start_lower_bound_us == 0
	    || timing.start_lower_bound_us > now
	    || now - timing.start_lower_bound_us >= srxl2::BusTimeoutUs) {
		// Never make delayed/buffered RC samples look newly received.
		++_unqualified_packets;
		_control_timing.invalidate();
		_check_timing = false;
		return;
	}

	const auto event = _endpoint.process(packet, timing.start_lower_bound_us);

	if (!event.accepted) {
		++_invalid_packets;
		_control_timing.invalidate();
		_check_timing = false;
		return;
	}

	if (event.channel_data) {
		if (_check_timing && _control_first_sequence != packet.first_sequence) {
			// A skipped timing observation must not alias a faster stream to 11 ms.
			_control_timing.invalidate();
		}

		_control_first_sequence = packet.first_sequence;
		_control_end_sequence = packet.last_sequence + 1;
		_control_length = packet.length;
		_check_timing = true;
		check_control_timing();
		publish(now);
	}

	if (event.reply.kind != srxl2::ReplyRequest::Kind::None) {
		if (_reply.kind != srxl2::ReplyRequest::Kind::None) {
			++_tx_skipped;
		}

		_reply = event.reply;
		_reply_first_sequence = packet.first_sequence;
		_reply_end_sequence = packet.last_sequence + 1;
		_reply_created = now;

		if (_reply.kind == srxl2::ReplyRequest::Kind::Telemetry) {
			uint8_t payload[srxl2::TelemetrySize] {};

			if (_param_telemetry_enabled.get() && !_control_timing.too_fast()) {
				_telemetry.nextPayload(now, payload);
			}

			_endpoint.make_telemetry(payload, _reply);
		}
	}
}

void Srxl2Rc::check_control_timing()
{
	if (!_check_timing) {
		return;
	}

	Srxl2Transport::Timing timing {};

	if (!_transport.packetTiming(_control_first_sequence, _control_end_sequence, timing)) {
		_control_timing.invalidate();
		_check_timing = false;
		return;
	}

	if (!timing.valid || timing.idle_observed_us == 0) {
		return;
	}

	const bool was_fast = _control_timing.too_fast();
	_control_timing.observe(_control_first_sequence, _control_end_sequence,
				timing.first_sequence, timing.end_sequence, timing.start_lower_bound_us,
				timing.idle_observed_us, _control_length);

	if (!was_fast && _control_timing.too_fast()) {
		PX4_WARN("control timing below 11 ms: telemetry disabled until restart");
	}

	_check_timing = false;
}

void Srxl2Rc::send_reply(hrt_abstime now)
{
	if (_reply.kind == srxl2::ReplyRequest::Kind::None) {
		return;
	}

	const bool handshake = _reply.kind == srxl2::ReplyRequest::Kind::Handshake;

	// Even a no-data telemetry packet occupies a reply slot, so suppress all
	// control replies if the observed bus timing is outside our supported set.
	if ((!handshake && (!_control_timing.qualified() || _control_timing.too_fast()))
	    || now - _reply_created >= PendingReplyTimeoutUs) {
		++_tx_skipped;
		_reply = {};
		return;
	}

	const auto result = _transport.tryTransmit(_reply.data, _reply.length,
			    _reply_first_sequence, _reply_end_sequence, handshake);

	if (result == Srxl2Transport::TxResult::Busy) {
		return;
	}

	if (result == Srxl2Transport::TxResult::Sent) {
		++_tx_sent;

	} else {
		++_tx_skipped;

		if (result == Srxl2Transport::TxResult::Error) {
			++_io_errors;
		}
	}

	_reply = {};
}

void Srxl2Rc::publish(hrt_abstime now)
{
	const auto state = _endpoint.channel_state(now);
	input_rc_s input {};
	input.timestamp = now;
	input.timestamp_last_signal = state.timestamp_last_signal;
	input.channel_count = state.channel_count;
	input.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_SRXL2;
	input.rc_lost = state.rc_lost;
	input.rc_failsafe = state.rc_failsafe;
	input.rssi = state.rssi;
	input.rssi_dbm = state.rssi_dbm;
	input.link_quality = -1;
	input.link_snr = -1;
	input.rc_lost_frame_count = state.lost_frame_count;
	input.rc_total_frame_count = state.total_frame_count;
	memcpy(input.values, state.values, sizeof(input.values));
	_input_rc_pub.publish(input);
	perf_count(_publish_perf);
	_published = true;
	_last_lost = state.rc_lost;
	_last_failsafe = state.rc_failsafe;
}

int Srxl2Rc::print_status()
{
	const auto state = _endpoint.channel_state(hrt_absolute_time());
	const auto &statistics = _parser.statistics();
	const auto &transport = _transport.counters();
	PX4_INFO("UART: %s, 115200, TX single-wire", _device);
	PX4_INFO("Receiver: 0x%02x, state: %u, channels: %u, lost: %s, failsafe: %s",
		 _endpoint.master_id(), static_cast<unsigned>(_endpoint.state()), state.channel_count,
		 state.rc_lost ? "yes" : "no", state.rc_failsafe ? "yes" : "no");
	PX4_INFO("Channel masks (PX4 order): initialized: 0x%05" PRIx32 ", updated in 100 ms: 0x%05" PRIx32,
		 state.valid_mask, state.fresh_mask);
	PX4_INFO("Receiver frame losses: %u, failsafe holds: %u",
		 static_cast<unsigned>(state.lost_frame_count), static_cast<unsigned>(state.hold_count));
	PX4_INFO("RX bytes: %" PRIu32 ", packets: %" PRIu32 ", CRC errors: %" PRIu32 ", length errors: %" PRIu32,
		 _bytes_rx, statistics.packets, statistics.crc_errors, statistics.length_errors);
	PX4_INFO("Rejected packets: %" PRIu32 ", unqualified timing: %" PRIu32, _invalid_packets, _unqualified_packets);
	PX4_INFO("Replies sent: %" PRIu32 ", skipped: %" PRIu32 ", IO errors: %" PRIu32, _tx_sent, _tx_skipped, _io_errors);
	PX4_INFO("UART RX overruns: %" PRIu32 ", busy replies: %" PRIu32 ", stale replies: %" PRIu32 ", last error: %d",
		 transport.rx_overruns, transport.busy, transport.stale, _transport.lastError());
	PX4_INFO("Telemetry: %s, dropped text: %" PRIu32,
		 _control_timing.too_fast() ? "unsupported timing" : (!_control_timing.qualified() ? "qualifying timing" :
				 (_param_telemetry_enabled.get() ? "enabled" : "disabled")),
		 _telemetry.droppedMessages());
	perf_print_counter(_cycle_perf);
	perf_print_counter(_publish_perf);
	return PX4_OK;
}

int Srxl2Rc::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Srxl2Rc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Spektrum SRXL2 RC input and stock sensor/TextGen telemetry on a timed half-duplex
UART. Connect receiver signal to TX, with +5V and GND. Use 115200 and 11/22 ms RF.
Release the serial port from MAVLink before setting RC_SRXL2_PRT_CFG.
Software binding is not supported; use the receiver bind button.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("srxl2_rc", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("radio_control");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS5", "<file:dev>", "SRXL2 UART device", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_ERROR;
}

extern "C" __EXPORT int srxl2_rc_main(int argc, char *argv[])
{
	return ModuleBase::main(Srxl2Rc::desc, argc, argv);
}
