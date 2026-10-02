#!/usr/bin/env python3
"""Run the production driver with a task-group-scoped, timed fake UART.

POSIX shares descriptors across threads and cannot reproduce this NuttX bug.
The harness models separate launcher/worker descriptor tables and scheduling;
it compiles the actual driver methods and protocol, without UART hardware.
Timed packet replay also checks telemetry recovery when RF starts after boot.
"""

import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--root', type=Path)
parser.add_argument('--source', type=Path, help='Override driver source for negative regression check')
args = parser.parse_args()
root = args.root or Path(__file__).resolve().parents[5]
driver = root / 'src/drivers/rc/srxl2_rc'

stubs = r'''
#pragma once
#include <atomic>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "Srxl2.hpp"

using hrt_abstime = uint64_t;
inline uint64_t now_us = 100000;
inline int task_group = 0; // 0: launcher, 1: serial worker
inline unsigned opens = 0, closes = 0, reads = 0, wrong_group = 0;
inline unsigned open_failures = 0, read_error = 0, publications = 0;
inline unsigned perf_active = 0, telemetry_updates = 0;
inline uint64_t sequence = 0;
inline std::vector<uint8_t> wire;
inline std::vector<std::vector<uint8_t>> transmitted;
inline int owner_group = -1;
inline uint64_t hrt_absolute_time() { return now_us; }
using px4_guid_t = uint8_t[16];
inline void board_get_px4_guid(px4_guid_t &guid) { memset(guid, 0x31, sizeof(guid)); }
inline bool board_rc_conflicting(const char *) { return false; }
inline int px4_getopt(int argc, char **argv, const char *, int *index, const char **arg)
{
    if (*index + 1 >= argc) { return EOF; }
    assert(strcmp(argv[*index], "-d") == 0);
    *arg = argv[*index + 1]; *index += 2; return 'd';
}
template<typename... T> inline void log_message(const char *, T...) {}
#define PX4_ERR(...) log_message(__VA_ARGS__)
#define PX4_WARN(...) log_message(__VA_ARGS__)
#define PX4_INFO(...) log_message(__VA_ARGS__)
#define PRINT_MODULE_DESCRIPTION(...) log_message(__VA_ARGS__)
#define PRINT_MODULE_USAGE_NAME(...) log_message(__VA_ARGS__)
#define PRINT_MODULE_USAGE_SUBCATEGORY(...) log_message(__VA_ARGS__)
#define PRINT_MODULE_USAGE_COMMAND(...) log_message(__VA_ARGS__)
#define PRINT_MODULE_USAGE_PARAM_STRING(...) do {} while (0)
#define PRINT_MODULE_USAGE_DEFAULT_COMMANDS() do {} while (0)
#define MODULE_NAME "srxl2_rc"
#define PX4_OK 0
#define PX4_ERROR (-1)
#define __EXPORT
constexpr int task_id_is_work_queue = -2;

class ModuleBase {
public:
    struct Descriptor {
        Descriptor(int (*)(int, char **), int (*)(int, char **), int (*)(const char *)) {}
        std::atomic<ModuleBase *> object{nullptr};
        int task_id{-1};
    };
    virtual ~ModuleBase() = default;
    virtual int print_status() { return 0; }
    bool should_exit() const { return exiting; }
    void request_stop() { exiting = true; }
    static void exit_and_cleanup(Descriptor &d) {
        assert(task_group == 1);
        auto *object = d.object.exchange(nullptr); delete object; d.task_id = -1;
    }
    static int main(Descriptor &, int, char **) { return 0; }
    bool exiting{false};
};
class ModuleParams {
public:
    explicit ModuleParams(void *) {}
    virtual ~ModuleParams() = default;
    void updateParams() {}
};
namespace px4 {
inline int serial_port_to_wq(const char *) { return 1; }
class ScheduledWorkItem {
public:
    ScheduledWorkItem(const char *, int) {}
    virtual ~ScheduledWorkItem() { assert(!scheduled); }
    virtual void Run() = 0;
    void ScheduleNow() { scheduled = true; }
    void ScheduleOnInterval(unsigned value) { scheduled = true; interval = value; }
    void ScheduleClear() { scheduled = false; }
    bool scheduled{false}; unsigned interval{0};
};
}
struct input_rc_s {
    static constexpr uint8_t RC_INPUT_SOURCE_PX4FMU_SRXL2 = 16;
    uint64_t timestamp{}, timestamp_last_signal{};
    uint16_t values[18]{};
    uint32_t rc_lost_frame_count{}, rc_total_frame_count{};
    int32_t rssi{}; float rssi_dbm{};
    int link_quality{}, link_snr{};
    uint8_t channel_count{}, input_source{};
    bool rc_lost{}, rc_failsafe{};
};
inline input_rc_s last_input{};
struct parameter_update_s {};
#define ORB_ID(x) 0
namespace uORB {
template<typename T> class PublicationMulti {
public:
    explicit PublicationMulti(int) {}
    void publish(const T &data) { last_input = data; ++publications; }
};
class SubscriptionInterval {
public:
    SubscriptionInterval(int, unsigned) {}
    bool updated() { return false; }
    void copy(parameter_update_s *) {}
};
}
using perf_counter_t = int;
constexpr int PC_ELAPSED = 0, PC_INTERVAL = 1;
inline int perf_alloc(int, const char *) { return 1; }
inline void perf_free(int) {}
inline void perf_begin(int) { ++perf_active; }
inline void perf_end(int) { assert(perf_active > 0); --perf_active; }
inline void perf_count(int) {}
inline void perf_print_counter(int) {}
struct FakeParamBool { bool get() const { return true; } };
#define DEFINE_PARAMETERS(...) FakeParamBool _param_telemetry_enabled;
class Srxl2Telemetry {
public:
    void update(uint64_t) { ++telemetry_updates; }
    bool nextPayload(uint64_t, uint8_t payload[16]) { memset(payload, 0, 16); payload[0] = 0x7f; return true; }
    uint32_t droppedMessages() const { return 0; }
};
'''

harness = r'''
#include "Srxl2Rc.hpp"

static Srxl2Transport::Timing rx_timing;

bool Srxl2Transport::open(const char *)
{
    ++opens;
    if (open_failures) { --open_failures; _last_error = EBUSY; return false; }
    assert(owner_group == -1);
    owner_group = task_group; _fd = 3; _timed_enabled = true; return true;
}
void Srxl2Transport::close()
{
    if (_fd < 0) { return; }
    assert(owner_group == task_group);
    ++closes; owner_group = -1; _fd = -1; _timed_enabled = false;
}
ssize_t Srxl2Transport::read(uint8_t *buffer, size_t capacity, RxChunk &chunk)
{
    ++reads;
    if (owner_group != task_group) { ++wrong_group; _last_error = EBADF; return -1; }
    if (read_error) { _last_error = read_error; read_error = 0; return -1; }
    size_t count = wire.size() < capacity ? wire.size() : capacity;
    for (size_t i = 0; i < count; ++i) { buffer[i] = wire[i]; }
    wire.erase(wire.begin(), wire.begin() + count);
    chunk.first_sequence = sequence; sequence += count; chunk.end_sequence = sequence;
    if (count) {
        rx_timing = {};
        rx_timing.first_sequence = chunk.first_sequence;
        rx_timing.end_sequence = chunk.end_sequence;
        rx_timing.start_lower_bound_us = now_us - 3000;
        rx_timing.idle_observed_us = rx_timing.start_lower_bound_us
            + 100 + (count * 10000000 + 115199) / 115200 + 87;
        rx_timing.valid = true;
    }
    return count;
}
bool Srxl2Transport::packetTiming(uint64_t first, uint64_t end, Timing &timing)
{
    assert(owner_group == task_group);
    timing = rx_timing;
    return timing.valid && first >= timing.first_sequence && end <= timing.end_sequence;
}
Srxl2Transport::TxResult Srxl2Transport::tryTransmit(const uint8_t *data, size_t length,
        uint64_t first, uint64_t end, bool)
{
    assert(owner_group == task_group);
    assert(first == rx_timing.first_sequence && end == rx_timing.end_sequence);
    assert(srxl2::crc16(data, length) == 0);
    transmitted.emplace_back(data, data + length);
    return TxResult::Sent;
}
static Srxl2Rc *start()
{
    task_group = 0;
    char command[] = "start", option[] = "-d", device[] = "/dev/ttyS5";
    char *argv[] = {command, option, device};
    assert(Srxl2Rc::task_spawn(3, argv) == 0);
    auto *driver = static_cast<Srxl2Rc *>(Srxl2Rc::desc.object.load());
    assert(driver && driver->scheduled);
    return driver;
}
static void run(Srxl2Rc *driver)
{
    task_group = 1; assert(driver->scheduled); driver->Run();
    now_us += 11000; assert(perf_active == 0);
}
static void stop(Srxl2Rc *driver)
{
    task_group = 0; driver->request_stop();
    run(driver); assert(!Srxl2Rc::desc.object.load());
    assert(owner_group == -1);
}
static void finish_packet()
{
    wire[2] = wire.size() + 2;
    uint16_t crc = srxl2::crc16(wire.data(), wire.size());
    wire.push_back(crc >> 8); wire.push_back(crc);
}
static void controls(uint8_t mask = 15, uint8_t reply = 0)
{
    wire = {0xa6, 0xcd, 0, 0, reply, 83, 0, 0, mask, 0, 0, 0};
    for (unsigned i = 0; i < 8; ++i) {
        if (mask & (1u << i)) { wire.push_back(0); wire.push_back(0x80); }
    }
    finish_packet();
}
static void test_telemetry_recovery(uint64_t period)
{
    auto *driver = start();
    transmitted.clear();
    // Complete discovery while the transmitter is still off.
    wire = {0xa6, 0x21, 0, 0x10, srxl2::DeviceId, 10, 0, 3, 1, 0, 0, 0};
    finish_packet(); run(driver);
    assert(transmitted.size() == 1 && transmitted.back()[1] == 0x21);
    wire = {0xa6, 0x21, 0, 0x10, 0xff, 10, 0, 3, 1, 0, 0, 0};
    finish_packet(); run(driver);
    transmitted.clear();
    uint64_t packet_time = now_us;
    auto receive = [&](uint64_t interval, uint8_t mask, uint8_t reply = srxl2::DeviceId) {
        packet_time += interval; now_us = packet_time;
        controls(mask, reply); run(driver);
    };
    for (unsigned i = 0; i < 10; ++i) {
        receive(5500, 0);
        assert(last_input.rc_lost && transmitted.empty());
    }
    assert(driver->_control_timing.too_fast());

    // Turning the transmitter on restores RC immediately; telemetry needs
    // two complete supported intervals, without restarting the driver.
    receive(5500, 15);
    assert(!last_input.rc_lost && transmitted.empty());
    receive(period, 15);
    assert(transmitted.empty());
    receive(period, 15);
    assert(transmitted.size() == 1 && "telemetry remains disabled after late RF acquisition");
    assert(transmitted.back()[1] == 0x80 && transmitted.back()[3] == 0x10);
    assert(transmitted.back()[4] == 0x7f); // Actual payload, not an empty keepalive.
    receive(period, 15, 0); // A qualified bus is not permission to send unsolicited replies.
    assert(transmitted.size() == 1);

    // Repeated transmitter loss/recovery must also recover without restart.
    for (unsigned cycle = 0; cycle < 2; ++cycle) {
        transmitted.clear();
        for (unsigned i = 0; i < 20; ++i) { receive(5500, 0); }
        assert(last_input.rc_lost && transmitted.empty());
        receive(5500, 15);
        receive(period, 15);
        assert(!last_input.rc_lost && transmitted.empty());
        receive(period, 15);
        assert(transmitted.size() == 1 && transmitted.back()[4] == 0x7f);
    }
    stop(driver);
}
int main()
{
    // The old code opens in the launcher and then repeatedly reads EBADF.
    auto *driver = start();
    controls(); run(driver);
    assert(wrong_group == 0 && "UART descriptor belongs to the launcher, not the worker");
    assert(opens == 1 && owner_group == 1 && driver->interval == 250);
    assert(driver->_bytes_rx == 22 && driver->_io_errors == 0);
    assert(!last_input.rc_lost && last_input.channel_count == 4);
    assert(last_input.timestamp_last_signal != 0);
    assert(telemetry_updates > 0);
    run(driver); assert(opens == 1); // Do not reopen every polling cycle.
    stop(driver); assert(closes == 1 && last_input.rc_lost);

    // Stopping before the first scheduled run must not open the UART.
    driver = start(); stop(driver); assert(opens == 1 && closes == 1);

    // A busy/unavailable port must not leave a zombie polling task.
    open_failures = 1; unsigned previous_reads = reads;
    driver = start(); run(driver);
    assert(!Srxl2Rc::desc.object.load() && owner_group == -1);
    assert(reads == previous_reads);

    // Transient interruption preserves the owner; permanent IO failure
    // publishes RC lost, releases the port and cancels future polling.
    driver = start(); controls(); run(driver);
    read_error = EINTR; run(driver);
    assert(Srxl2Rc::desc.object.load() == driver && owner_group == 1);
    read_error = EIO; run(driver);
    assert(!Srxl2Rc::desc.object.load() && owner_group == -1 && last_input.rc_lost);

    // Restart reacquires a fresh descriptor in the worker's task group.
    driver = start(); controls(); run(driver);
    assert(!last_input.rc_lost && driver->_io_errors == 0);
    stop(driver);
    assert(opens == 4 && closes == 3 && perf_active == 0);
    test_telemetry_recovery(11000);
    test_telemetry_recovery(22000);
    puts("SRXL2 driver: worker-owned UART, RC input, startup failure, stop/restart and IO failure checks passed");
    puts("SRXL2 telemetry: late transmitter startup and repeated RF recovery at 11/22 ms passed");
}
'''

with tempfile.TemporaryDirectory(prefix='srxl2-lifecycle-') as directory:
    temp = Path(directory)
    (temp / 'stubs.hpp').write_text(stubs)
    # Keep the production class declaration; substitute platform dependencies
    # and expose private methods only to this test harness.
    header = (driver / 'Srxl2Rc.hpp').read_text()
    header = '\n'.join(line for line in header.splitlines() if not line.startswith('#include'))
    header = '#include "stubs.hpp"\n#include "Srxl2Transport.hpp"\n' + header.replace('private:', 'public:')
    (temp / 'Srxl2Rc.hpp').write_text(header)
    source = (args.source or driver / 'Srxl2Rc.cpp').read_text()
    source = '\n'.join(line for line in source.splitlines()
                       if not line.startswith('#include <px4_platform_common/'))
    (temp / 'Srxl2Rc.cpp').write_text(source)
    (temp / 'test.cpp').write_text(harness)
    compiler = shutil.which('clang++') or shutil.which('g++')
    assert compiler, 'C++ compiler required'
    subprocess.run([compiler, '-std=c++17', '-g', '-O1', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                    '-I' + str(temp), '-I' + str(driver), '-I' + str(root / 'src/lib/rc/srxl2'),
                    str(temp / 'Srxl2Rc.cpp'), str(temp / 'test.cpp'),
                    str(root / 'src/lib/rc/srxl2/Srxl2.cpp'), '-o', str(temp / 'test')], check=True)
    subprocess.run([str(temp / 'test')], check=True)
