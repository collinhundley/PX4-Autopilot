# Pixhawk 6C USB Ethernet (Experimental)

The custom `px4_fmu-v6c_osd` target exposes CDC-ACM serial and CDC-NCM 1.0 Ethernet together on its USB-C connector.
`px4_fmu-v6c_default` and the bootloader retain their existing USB configuration.
This is a bench-test implementation; iPhone MAVLink communication, flashing recovery and flight-load performance require the hardware checks below before release.

## USB Configuration

The application presents ACM on interfaces 0–1 (endpoints IN1, IN2, OUT3) and NCM on interfaces 2–3 (IN4, IN5, OUT6), with interface association descriptors.
The VID/PID (`3185:0038`), manufacturer/product strings and existing serial string behaviour are preserved.
The composite application uses `bcdDevice=0x0200` to distinguish its descriptor revision.
Check both previously connected and fresh desktop hosts for cached driver associations.

ACM remains `/dev/ttyACM0` in PX4.
The bootloader is not modified by this feature and still provides the normal serial firmware-upload path.
Both application functions enumerate simultaneously; there is no host identification, serial-first timeout, or class switching.
USB class switching would require disconnection and re-enumeration, interrupting active connections.
USB enumeration alone does not identify an iPhone reliably.

The NCM driver uses full-speed USB, NTB16, a 1,500-byte IP MTU, two fixed 2 KiB transfer buffers, one transmitted Ethernet frame per NTB and at most 32 received frames per NTB.
The device and host receive distinct, stable, locally administered MAC addresses derived from the board UID.
NCM 1.0 follows [Apple's Accessory Design Guidelines, section 38](https://developer.apple.com/accessories/Accessory-Design-Guidelines.pdf).
ECM, RNDIS, MBIM, TCP, IPv6, DNS resolution and routing are not enabled for this feature.

## Network Contract

| Setting | Value |
| --- | --- |
| PX4 interface | `eth0`, reserved for USB on this target |
| PX4 address | `192.168.203.1/30` |
| Host address | `192.168.203.2/30`, one DHCP lease |
| Gateway and DNS | Neither advertised |
| PX4 UDP port | `14550` |
| App UDP port | `14550` |
| PX4 destination | Unicast `192.168.203.2:14550` |
| Streams | Existing `USB_MAV_MODE` |
| Initial transmit limit | `100000` bytes/second |
| Start transmitting | After receiving the app's first MAVLink message |

The foreground iOS app should use `Network.framework` UDP sockets on the wired Ethernet interface, bind local port 14550 and send MAVLink heartbeats to `192.168.203.1:14550`.
Send heartbeats periodically so startup remains reliable if DHCP or firmware services are still starting when the first packet is sent.
Handle path changes, cancel stale connections and recreate the socket after cable reconnect or app restart.
No iOS application is included in this repository.

The app still requires normal Local Network permission and a suitable `NSLocalNetworkUsageDescription`; see [Apple TN3179](https://developer.apple.com/documentation/technotes/tn3179-understanding-local-network-privacy).
Unicast avoids multicast discovery and its entitlement.
Keeping the link without a default gateway or DNS server allows Wi-Fi/cellular Internet connectivity to coexist, subject to the hardware tests below.
Continuous background operation is outside the scope of this firmware change.

## Startup And Lifetime

`cdcacm_autostart` owns composite initialisation and the USB network services.
Its `connect` command provides the same initialisation for startup recovery.
Ethernet servicing is independent of whether a host opens the ACM serial function.
`SYS_USB_AUTO` and serial `USB_MAV_MODE` behaviour are retained: `0` leaves the serial device available, `1` detects serial protocols, and `2` starts serial MAVLink.
Ethernet operates in all three modes.
The existing negative `SYS_USB_AUTO` startup setting disables the manager and therefore this network feature too.

The composite device and `eth0` remain registered across ordinary cable disconnects.
NCM reset/alternate-setting/carrier changes control link availability.
One DHCP daemon is retained across reconnects, and the manager starts/stops its UDP MAVLink instance on port 14550 as carrier changes.
Reserve that port for the manager; do not manually start a competing MAVLink instance on it.
Network configuration or service failures leave ACM registered.
Composite builds serialize MAVLink CLI startup so simultaneous serial and UDP startup cannot race shared initialization.
A UDP startup timeout does not cancel the spawned task: the manager retains ownership of that attempt, avoids duplicate launches and can stop a late instance on link loss.
If startup remains unconfirmed or a service cannot stop, inspect `cdcacm_autostart status` and restart the disarmed board before retrying; ACM remains registered.

DHCP and MAVLink CLI operations execute in a short-lived service task, outside PX4's shared `lp_default` queue.
The DHCP daemon uses a one-second receive timeout and two-second start/stop deadlines.
Driver receive processing uses NuttX's existing single LPWORK thread; teardown drains its pending callback before releasing memory.
The manager will retain the composite device if stopping a service fails, if a managed NSH/passthrough session still owns ACM, or while armed.
This protects open serial sessions and preserves the existing prohibition on application-driven USB reconfiguration while armed.
A normal disarmed stop waits for its serial MAVLink stop before freeing the composite device.
Physical unplug/replug and host-driven USB resets remain supported.

Use these existing commands for inspection:

```sh
cdcacm_autostart status
mavlink status
free
top
work_queue status
```

## Source And Build Validation

The NCM class is based on Apache NuttX commit `4e79741e7db15b74b9df78219b8c9e3927689410`, adapted to this fork's `net_driver_s` API.
It preserves the pinned NuttX SRXL2 changes and does not import the newer network lower-half framework.
The adaptation validates complete NTBs before dispatch, separates interrupt/TX request ownership, submits EP0 replies through the composite driver, and reasserts software connection after reset.
The NuttX apps change adds bounded DHCP daemon lifecycle operations and validates received DHCP option boundaries.
Both submodules must accompany the PX4 changes.
Before distributing the PX4 branch, publish the dependency commits to accessible forks, verify the configured submodule URLs can fetch them, and complete the remaining validation.

Build using the repository's macOS Docker workflow with image `px4io/px4-dev:v1.17.0`.
After changing between NuttX configurations, clean generated NuttX libraries as well as the selected CMake build outputs.
This pinned build system can otherwise reuse archive members built against the previous configuration.
Do not infer a successful composite build from package creation alone: verify that the link map retains `composite_initialize`, `cdcncm_work` and `dhcpd_start` in flash.

Run the host checks from the repository root:

```sh
python3 Tools/usb_ncm/run_tests.py
python3 Tools/usb_ncm/test_size.py
python3 Tools/usb_ncm/check_size.py build/px4_fmu-v6c_osd
```

The protocol tests compile the actual backported codec and driver against a deterministic USB/LPWORK model with AddressSanitizer and UndefinedBehaviorSanitizer.
They cover descriptor interfaces/endpoints, NCM negotiation, packet filters, alternate settings, notifications, transfer-buffer ownership, pending-transfer reset and work cancellation at teardown.
Codec checks include all packet lengths, truncated transfers, invalid offsets/lengths, descriptor/datagram overlap, chained/cyclic tables and random input.
These tests do not simulate the STM32 USB peripheral or prove host enumeration.

The application flash limit is 1,966,080 bytes (1,920 KiB) at `0x08020000`, after the 128 KiB bootloader reservation.
The packager rejects raw images exceeding the prototype's limit.
The OSD build also checks ELF flash load addresses, raw binary size and the decompressed `.px4` image, verifies identical payloads, and reports whether at least 16 KiB remains.
Compressed package size is not an acceptance measure.
Existing OSD flight features and the linker flash region are unchanged.

## Initial Software Validation

The bench build was made on `feat/usb-ncm`, based on PX4 `4f7a87b2f758daf07e3bafa9bbee3e4315ea2a54`, with local NuttX and NuttX apps changes.
The bench build's dependency bases are NuttX `c7d90a1b5ff3adb7e6485f4607dbee078e14a458` and NuttX apps `e37940d8535f603a16b8f6f21c21edaf584218aa`.
The current source checkpoint pins the local NuttX commit `08fa343b1e` and NuttX apps commit `4e972bf36`, capturing those tested changes.
These dependency commits are not yet published; the physical feasibility gate remains open pending iPhone MAVLink testing.
The source snapshot and build logs are saved with the bench artifact; the base commit alone does not reproduce a dirty build.

| Measurement | Fresh Baseline | Composite Build |
| --- | ---: | ---: |
| ELF flash load span | 1,851,084 bytes | 1,878,144 bytes |
| Raw application binary | 1,851,084 bytes | 1,878,144 bytes |
| Decompressed PX4 image | 1,851,084 bytes | 1,878,144 bytes |
| Spare application flash | 114,996 bytes | 87,936 bytes |
| Linker AXI SRAM allocation | 70,112 bytes | 73,916 bytes |

The flash increase is 27,060 bytes; the linker-reported static RAM increase is 3,804 bytes.
The composite build retains more than the desired 16 KiB flash reserve.
The NTB16, mocked driver, DHCP lifecycle/parser, USB service lifecycle and five firmware-size regression tests pass, as do the repository formatting checks.
The mock tests use sanitizers; hardware validation is partial, as recorded below.

The bootloader regression build passes at 46,300 bytes of flash and 7,284 bytes of static SRAM.
The existing checked-in bootloader binary is preserved; it is not replaced or flashed by this change.

The default firmware build gate is blocked by an existing flash overflow.
The modified default configuration links to 1,976,880 bytes and fails by 10,800 bytes; a control build using original `main` PX4 sources links to 1,976,728 bytes and already fails by 10,648 bytes.
The control uses the unchanged serial-only default configuration, with the local NCM/DHCP submodule additions disabled.
No default flight features or linker limits were changed to hide this failure.
Resolve and rebuild that target separately before declaring every release build gate passed.

## Hardware Acceptance Record

The initial test pair is a Pixhawk 6C and an iPhone 16 Pro Max running iOS 27, as reported by the owner.
Record the exact iOS build, board revision, firmware/submodule revisions, artifact SHA-256, cable/adaptor and host OS versions when testing.

On 2026-10-03, the owner reported the following results after flashing the new bench firmware:

| Check | Reported Result |
| --- | --- |
| Firmware installation on Pixhawk 6C | Successful |
| macOS serial MAVLink in *QGroundControl* | Working with only the serial connection enabled |
| macOS UDP MAVLink in *QGroundControl* | Working with only the UDP connection enabled |
| iPhone Ethernet enumeration | **Ethernet** appears in Settings, listing `PX4 FMU v6C.x` |
| iPhone IPv4 address | `192.168.203.2` |
| iPhone MAVLink exchange | Not yet tested |

These are owner-reported bench observations; the exact flashed artifact filename/hash and host build versions were not supplied with this report.
macOS serial and UDP were exercised separately, so concurrent communication remains untested.
Successful installation does not yet establish application reboot-to-bootloader or repeat-upload recovery.
The early feasibility gate remains open until bidirectional iPhone MAVLink communication is demonstrated.

Complete the remaining checks below before release:

1. With the vehicle disarmed and props removed, load the bench firmware through the existing bootloader.
   Confirm both ACM and NCM interfaces enumerate simultaneously on a desktop.
   Connect the iPhone and confirm wired Ethernet appears, DHCP assigns `192.168.203.2/30`, and the app exchanges MAVLink heartbeats over UDP.
   This is the early feasibility gate; do not treat the feature as release-ready before it passes.
2. Transfer parameters and a mission in both directions.
   Repeat app restart, cable reconnect and lock/unlock recovery, including Wi-Fi and cellular enabled.
3. On macOS, Linux and Windows, verify serial MAVLink and NSH, including a host without an NCM driver.
   Exercise serial and Ethernet simultaneously where supported.
   Include a previously connected host and a fresh host.
4. Verify application reboot-to-bootloader, normal serial firmware upload, and power-cycle recovery with networking enabled.
5. Repeat reconnects and sustained traffic with sensors, logging, OSD and SRXL2 active.
   Record heap use, task/LPWORK stack high-water marks, CPU load, dropped packets and control-loop scheduling.
   Verify that resource use stabilises and that no task hangs or scheduling regressions occur.
6. Verify disarmed manager stop/restart and reset during pending transfers.
   Check service-allocation failure and occupied UDP port behaviour while ACM remains usable.
7. Record successful fresh OSD, default and bootloader builds and the strict OSD size report.

Static RAM from the ELF does not include dynamically allocated class state, transfer buffers, socket buffers, service tasks or MAVLink instances.
The compiled NCM class allocation is 1,876 bytes, including its Ethernet frame buffer and metadata.
The two NTB buffers, EP0 buffer and notification buffer add 4,240 bytes, for 6,116 bytes before controller request objects, composite state and allocator overhead.
The configured DHCP stack is 2,048 bytes and the short-lived service-task stack is 2,600 bytes, in addition to existing MAVLink task allocations.
The existing NuttX LPWORK stack is 1,632 bytes; its high-water mark must be measured with NCM traffic.
Actual stack and CPU costs require the bench stress test; no iPhone compatibility or runtime-performance guarantee is made from host tests alone.
