/* SPDX-License-Identifier: BSD-3-Clause */
#pragma once
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <arpa/inet.h>
#include <net/if.h>

namespace px4
{
template<typename T> class atomic : public std::atomic<T>
{
public:
	using std::atomic<T>::atomic;
	bool compare_exchange(T *expected, T value) { return this->compare_exchange_strong(*expected, value); }
};
}

static void test_log(const char *, ...) {}
#define PX4_INFO(...) test_log(__VA_ARGS__)
#define PX4_WARN(...) test_log(__VA_ARGS__)
#define PX4_ERR(...) test_log(__VA_ARGS__)
#define ORB_ID(x) 0
#define SCHED_DEFAULT 0
#define SCHED_PRIORITY_DEFAULT 100
#define BOARDIOC_USBDEV_COMPOSITE 3
#define BOARDIOC_USBDEV_INITIALIZE 0
#define BOARDIOC_USBDEV_CONNECT 1
#define BOARDIOC_USBDEV_CONTROL 7
struct boardioc_usbdev_ctrl_s {int usbdev; int action; int instance; int config; void **handle;};
struct actuator_armed_s {bool armed;};
static bool test_armed;
namespace uORB
{
class Subscription
{
public:
	explicit Subscription(int) {}
	void copy(actuator_armed_s *value) { value->armed = test_armed; }
};
}

static bool test_port_present;
extern "C" bool mavlink_network_port_exists(unsigned long port)
{
	assert(port == 14550);
	return test_port_present;
}

using test_entry_t = int (*)(int, char *[]);
static test_entry_t test_pending_task;
static bool test_spawn_fail;
static int test_starts, test_stops, test_dhcp_starts, test_dhcp_stops, test_unbinds;
static int test_dhcp_error, test_network_error;
static bool test_delayed_start, test_stop_fail;
static uint8_t test_flags = IFF_UP | IFF_RUNNING;

static int px4_task_spawn_cmd(const char *, int, int, int, test_entry_t entry, char *[])
{
	assert(!test_pending_task);
	if (test_spawn_fail) { return -1; }
	test_pending_task = entry;
	return 123;
}

static int boardctl(int command, uintptr_t arg)
{
	assert(command == BOARDIOC_USBDEV_CONTROL);
	auto *ctrl = reinterpret_cast<boardioc_usbdev_ctrl_s *>(arg);
	if (ctrl->action == BOARDIOC_USBDEV_CONNECT) { *ctrl->handle = ctrl; }
	return 0;
}

static void composite_uninitialize(void *) { ++test_unbinds; }
static int netlib_set_ipv4addr(const char *name, const in_addr *addr)
{
	assert(!strcmp(name, "eth0"));
	assert(ntohl(addr->s_addr) == 0xc0a8cb01);
	return test_network_error;
}
static int netlib_set_ipv4netmask(const char *, const in_addr *mask)
{
	assert(ntohl(mask->s_addr) == 0xfffffffc);
	return 0;
}
static int netlib_ifup(const char *) { return 0; }
static int netlib_getifstatus(const char *, uint8_t *flags) { *flags = test_flags; return 0; }
static int dhcpd_start(const char *) { ++test_dhcp_starts; return test_dhcp_error; }
static int dhcpd_stop() { ++test_dhcp_stops; return test_dhcp_error; }

extern "C" int mavlink_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "start")) {
		assert(argc == 13 && !strcmp(argv[2], "-u") && !strcmp(argv[3], "14550"));
		assert(!strcmp(argv[4], "-o") && !strcmp(argv[5], "14550"));
		assert(!strcmp(argv[6], "-t") && !strcmp(argv[7], "192.168.203.2"));
		assert(!strcmp(argv[10], "-r") && !strcmp(argv[11], "100000") && !strcmp(argv[12], "-w"));
		++test_starts;
		if (test_delayed_start) { return -1; }
		test_port_present = true;
		return 0;
	}
	assert(argc == 4 && !strcmp(argv[1], "stop"));
	++test_stops;
	if (test_stop_fail) { return -1; }
	if (!strcmp(argv[2], "-u")) {
		assert(!strcmp(argv[3], "14550"));
		if (!test_port_present) { return -1; }
		test_port_present = false;
	} else {
		assert(!strcmp(argv[2], "-d") && !strcmp(argv[3], "/dev/ttyACM0"));
	}
	return 0;
}
