/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

// Query registration under the MAVLink module lock, without exposing an
// instance pointer whose lifetime could end immediately after the query.
extern "C" bool mavlink_network_port_exists(unsigned long port);
