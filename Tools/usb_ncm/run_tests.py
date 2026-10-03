#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Build the actual NCM driver/codec against a deterministic host USB model."""
import os
from pathlib import Path
import subprocess
import tempfile

here = Path(__file__).resolve().parent
repo = here.parents[1]
driver = repo / 'platforms/nuttx/NuttX/nuttx/drivers/usbdev'
headers = ('config.h', 'irq.h', 'kmalloc.h', 'net/netdev.h', 'net/arp.h',
           'net/ip.h', 'semaphore.h', 'usb/usbdev.h', 'usb/cdc.h',
           'usb/cdcncm.h', 'wqueue.h')
with tempfile.TemporaryDirectory(prefix='px4-ncm-tests-') as temp:
    temp = Path(temp)
    for header in headers:
        p = temp / 'nuttx' / header
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('#include "ncm_test_stubs.h"\n')
    (temp / 'netutils').mkdir()
    (temp / 'netutils/netlib.h').write_text('')
    dhcp = repo / 'platforms/nuttx/NuttX/apps/netutils/dhcpd'
    for source in ('test_ntb.c', 'test_driver.c', 'test_dhcp.c'):
        exe = temp / source[:-2]
        subprocess.run([os.environ.get('CC', 'clang'), '-std=c11', '-g',
                        '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', '-Wno-cpp',
                        '-fsanitize=address,undefined', f'-I{temp}', f'-I{here}',
                        f'-I{driver}', f'-I{dhcp}', str(here / source), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)

    # Compile the actual PX4 service helper with task/socket/module boundaries
    # mocked; no device or scheduler implementation is simulated here.
    network = temp / 'network'
    for header in ('px4_platform_common/log.h', 'px4_platform_common/tasks.h',
                   'px4_platform_common/atomic.h', 'modules/mavlink/mavlink_network.h',
                   'uORB/Subscription.hpp', 'uORB/topics/actuator_armed.h',
                   'nuttx/usb/composite.h', 'sys/boardctl.h',
                   'netutils/netlib.h', 'netutils/dhcpd.h'):
        p = network / header
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('#include "network_test_stubs.h"\n')
    exe = temp / 'test_network'
    subprocess.run([os.environ.get('CXX', 'clang++'), '-std=c++17', '-g',
                    '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
                    '-fsanitize=address,undefined', f'-I{network}', f'-I{here}',
                    f'-I{repo / "src/drivers/cdcacm_autostart"}',
                    str(here / 'test_network.cpp'), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
