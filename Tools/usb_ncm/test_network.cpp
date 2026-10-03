/* SPDX-License-Identifier: BSD-3-Clause */
#include "network_test_stubs.h"
#include "usb_network.cpp"

static void run_service()
{
	assert(test_pending_task);
	auto entry = test_pending_task;
	test_pending_task = nullptr;
	assert(entry(0, nullptr) == 0);
}

static void reset_test()
{
	assert(!test_pending_task);
	busy.store(false); closing.store(false); connected.store(false); network_ready.store(false);
	want_link.store(false); udp_owned.store(false); stop_serial_on_close.store(false); keep_serial_on_close.store(false);
	composite_handle = nullptr; dhcp_running = false;
	test_armed = test_port_present = test_spawn_fail = test_delayed_start = test_stop_fail = false;
	test_starts = test_stops = test_dhcp_starts = test_dhcp_stops = test_unbinds = 0;
	test_dhcp_error = test_network_error = 0;
	test_flags = IFF_UP | IFF_RUNNING;
}

int main()
{
	reset_test();
	assert(usb_network_connect() == 0);
	usb_network_update(true, 2);
	assert(test_starts == 0); // Work-queue caller never runs blocking commands.
	usb_network_update(true, 2); // One service task while busy.
	run_service();
	assert(test_starts == 1 && test_dhcp_starts == 1);
	usb_network_update(false, 2); run_service();
	assert(test_stops == 1 && test_dhcp_stops == 0 && test_unbinds == 0);
	assert(usb_network_connected()); // Keep device, interface and DHCP on unplug.
	usb_network_update(true, 2); run_service();
	assert(test_starts == 2);
	usb_network_stop(true, false); run_service();
	assert(test_stops == 3 && test_dhcp_stops == 1 && test_unbinds == 1);
	assert(!usb_network_connected());

	reset_test(); // An external instance on the reserved port is never stopped.
	assert(usb_network_connect() == 0); test_port_present = true;
	usb_network_update(true, 2); run_service();
	assert(test_starts == 0 && !udp_owned.load());
	usb_network_stop(); run_service(); assert(test_stops == 0);

	reset_test(); // CLI timeout does not mean its child was cancelled.
	assert(usb_network_connect() == 0); test_delayed_start = true;
	usb_network_update(true, 2); run_service();
	usb_network_update(true, 2); assert(!test_pending_task && test_starts == 1);
	usb_network_update(false, 2); run_service(); // Stop cannot yet find it.
	assert(udp_owned.load());
	test_port_present = true; // Child registers after the caller timed out.
	usb_network_update(false, 2); run_service();
	assert(!test_port_present && !udp_owned.load());
	usb_network_stop(); run_service(); assert(test_unbinds == 1);

	reset_test(); // Stop while a service is pending must still run teardown.
	assert(usb_network_connect() == 0);
	usb_network_update(true, 2); usb_network_stop(); run_service();
	assert(test_starts == 0 && test_unbinds == 1);

	reset_test(); // Failed task allocation is retryable, without blocking ACM.
	assert(usb_network_connect() == 0); test_spawn_fail = true;
	usb_network_update(true, 2); assert(!test_pending_task && !busy.load());
	test_spawn_fail = false; usb_network_update(true, 2); run_service();
	assert(test_starts == 1);
	test_armed = true; usb_network_stop(); run_service();
	assert(test_unbinds == 0 && usb_network_connected());
	test_armed = false; usb_network_stop(false, true); run_service();
	assert(test_unbinds == 0); // Retain an NSH/passthrough serial owner.
	usb_network_stop(); run_service(); assert(test_unbinds == 1);

	reset_test(); // Network or DHCP errors must retain usable ACM registration.
	test_network_error = -1; assert(usb_network_connect() == 0);
	usb_network_update(true, 2); assert(!test_pending_task && usb_network_connected());
	usb_network_stop(); run_service();
	reset_test(); assert(usb_network_connect() == 0); test_dhcp_error = -1;
	usb_network_update(true, 2); run_service(); assert(test_starts == 1);
	usb_network_stop(); run_service(); assert(test_unbinds == 0);
	test_dhcp_error = 0; usb_network_stop(); run_service(); assert(test_unbinds == 1);
	puts("USB service: deferred lifecycle, reconnect, ownership, late startup and failure retention passed");
}
