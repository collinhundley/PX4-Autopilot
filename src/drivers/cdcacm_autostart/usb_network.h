/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Connect runs in command/task context. Update/stop never block a work queue.
int usb_network_connect();
bool usb_network_connected();
void usb_network_update(bool vbus, int mode);
void usb_network_stop(bool stop_serial = false, bool keep_serial = false);
void usb_network_status();
