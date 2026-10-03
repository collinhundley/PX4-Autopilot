/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "usb_network.h"
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/atomic.h>
#include <modules/mavlink/mavlink_network.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <nuttx/usb/composite.h>
#include <sys/boardctl.h>
#include <netutils/netlib.h>
#include <netutils/dhcpd.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <errno.h>

extern "C" int mavlink_main(int argc, char *argv[]);

namespace
{
constexpr const char *Interface = "eth0";
constexpr const char *DeviceAddress = "192.168.203.1";
constexpr const char *HostAddress = "192.168.203.2";
constexpr const char *Netmask = "255.255.255.252";
constexpr const char *Port = "14550";
constexpr unsigned PortNumber = 14550;
px4::atomic<bool> busy{false};
px4::atomic<bool> closing{false};
px4::atomic<bool> connected{false};
px4::atomic<bool> network_ready{false};
px4::atomic<bool> want_link{false};
px4::atomic<bool> udp_owned{false};
px4::atomic<bool> stop_serial_on_close{false};
px4::atomic<bool> keep_serial_on_close{false};
px4::atomic<int> mavlink_mode{2};
void *composite_handle = nullptr; // Accessed only while busy is held.
bool dhcp_running = false;
void schedule_service();

bool vehicle_armed()
{
	uORB::Subscription armed_sub{ORB_ID(actuator_armed)};
	actuator_armed_s armed{};
	armed_sub.copy(&armed);
	return armed.armed;
}

int stop_udp()
{
	char *argv[] {(char *)"mavlink", (char *)"stop", (char *)"-u", (char *)Port, nullptr};
	return mavlink_main(4, argv);
}

void cleanup()
{
	if (vehicle_armed()) {
		PX4_WARN("Retaining USB composite while armed");
		closing.store(false);
		return;
	}

	if (udp_owned.load()) {
		if (stop_udp() != 0) {
			PX4_ERR("USB UDP stop failed; retaining interface");
			closing.store(false);
			return;
		}

		udp_owned.store(false);
	}

	if (dhcp_running) {
		if (dhcpd_stop() != 0) {
			PX4_ERR("USB DHCP stop failed; retaining interface");
			closing.store(false);
			return;
		}

		dhcp_running = false;
	}

	if (stop_serial_on_close.load()) {
		// Join the serial stop before freeing the composite's ACM requests.
		char *argv[] {(char *)"mavlink", (char *)"stop", (char *)"-d", (char *)"/dev/ttyACM0", nullptr};

		if (mavlink_main(4, argv) != 0) {
			PX4_WARN("Retaining USB after serial stop failure");
			closing.store(false);
			return;
		}
	}

	if (keep_serial_on_close.load()) {
		// NSH/passthrough still owns an open tty. It must not reference freed
		// ACM storage. Reuse the registered device when the manager restarts.
		PX4_INFO("Retaining USB for active serial session");
		closing.store(false);
		return;
	}

	if (composite_handle) {
		composite_uninitialize(composite_handle);
		composite_handle = nullptr;
	}

	connected.store(false);
	network_ready.store(false);
	closing.store(false);
}

int service_task(int, char *[])
{
	if (closing.load()) {
		cleanup();

	} else if (want_link.load() && !udp_owned.load()) {
		// DHCP and the MAVLink CLI can wait for other tasks. They must not
		// execute on lp_default, which also services other PX4 modules.
		// start() is idempotent and also recovers an exited daemon.
		{
			const bool dhcp_ready = dhcpd_start(Interface) == 0;
			dhcp_running = true; // A timed-out start may still need stopping.

			if (!dhcp_ready) {
				PX4_ERR("USB DHCP start failed");
			}
		}

		// Check the reserved port before acquiring ownership. A start timeout
		// does not cancel the spawned MAVLink task, so retain ownership even
		// if registration is delayed. Never launch a duplicate on a retry.
		if (mavlink_network_port_exists(PortNumber)) {
			PX4_ERR("USB UDP port %u already occupied", PortNumber);

		} else {
			// A DHCP failure still permits a manually addressed host using UDP.
			char mode[12];
			snprintf(mode, sizeof(mode), "%d", mavlink_mode.load());
			char *argv[] {(char *)"mavlink", (char *)"start", (char *)"-u", (char *)Port,
				      (char *)"-o", (char *)Port, (char *)"-t", (char *)HostAddress,
				      (char *)"-m", mode, (char *)"-r", (char *)"100000", (char *)"-w", nullptr
				     };
			udp_owned.store(true);

			if (mavlink_main(13, argv) != 0) {
				PX4_ERR("USB UDP startup unconfirmed; retaining ownership");
			}
		}

	} else if (!want_link.load() && udp_owned.load()) {
		if (stop_udp() == 0) {
			udp_owned.store(false);

		} else {
			PX4_ERR("USB UDP stop failed");
		}
	}

	// Recheck after releasing busy: a stop request arriving while this task
	// was active must still arrange teardown, even after its owner exits.
	busy.store(false);

	if (closing.load()) {
		schedule_service();
	}

	return 0;
}

void schedule_service()
{
	bool expected = false;

	if (!busy.compare_exchange(&expected, true)) {
		return;
	}

	if (px4_task_spawn_cmd("usb_net", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT - 30,
			       2600, service_task, nullptr) < 0) {
		busy.store(false);
		PX4_ERR("USB network service task allocation failed");
	}
}
}

int usb_network_connect()
{
	bool expected = false;

	if (closing.load() || !busy.compare_exchange(&expected, true)) {
		return -EBUSY;
	}

	if (connected.load()) {
		busy.store(false);
		return 0;
	}

	if (vehicle_armed()) {
		busy.store(false);
		return -EBUSY;
	}

	boardioc_usbdev_ctrl_s ctrl{};
	ctrl.usbdev = BOARDIOC_USBDEV_COMPOSITE;
	ctrl.action = BOARDIOC_USBDEV_INITIALIZE;
	ctrl.instance = 0;
	ctrl.config = 0;
	ctrl.handle = &composite_handle;
	int ret = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);

	if (ret == 0) {
		ctrl.action = BOARDIOC_USBDEV_CONNECT;
		ret = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
	}

	if (ret == 0) {
		in_addr ip{}, mask{};
		inet_aton(DeviceAddress, &ip);
		inet_aton(Netmask, &mask);

		if (netlib_set_ipv4addr(Interface, &ip) != 0 ||
		    netlib_set_ipv4netmask(Interface, &mask) != 0 || netlib_ifup(Interface) != 0) {
			// A network setup error must leave the serial function available.
			PX4_ERR("USB IPv4 configuration failed");

		} else {
			network_ready.store(true);
		}

		connected.store(true);
	}

	busy.store(false);
	return ret;
}

bool usb_network_connected()
{
	return connected.load();
}

void usb_network_update(bool vbus, int mode)
{
	uint8_t flags = 0;
	bool link = vbus && network_ready.load() && !closing.load() &&
		    netlib_getifstatus(Interface, &flags) == 0 && (flags & IFF_RUNNING) && (flags & IFF_UP);
	mavlink_mode.store(mode);
	want_link.store(link);

	if (connected.load() && (closing.load() || link != udp_owned.load())) {
		schedule_service();
	}
}

void usb_network_stop(bool stop_serial, bool keep_serial)
{
	stop_serial_on_close.store(stop_serial);
	keep_serial_on_close.store(keep_serial);
	closing.store(true);
	want_link.store(false);
	schedule_service();
}

void usb_network_status()
{
	uint8_t flags = 0;
	const bool carrier = netlib_getifstatus(Interface, &flags) == 0 && (flags & IFF_RUNNING);
	const bool udp_present = mavlink_network_port_exists(PortNumber);
	PX4_INFO("USB NCM: %s, UDP: %s%s", connected.load() ? "registered" : "unavailable",
		 udp_owned.load() ? (udp_present ? "registered" : "startup unconfirmed") : "unmanaged",
		 busy.load() ? " (service pending)" : "");
	PX4_INFO("USB carrier: %s, IPv4 setup: %s", carrier ? "up" : "down", network_ready.load() ? "ready" : "failed");
	PX4_INFO("USB IPv4: %s/30, peer %s, UDP %s", DeviceAddress, HostAddress, Port);
}
