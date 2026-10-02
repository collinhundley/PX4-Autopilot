# Spektrum SRXL2 (Pixhawk 6C)

The `srxl2_rc` driver provides RC input and standard Spektrum telemetry for an SPM4651T connected to a Pixhawk 6C TELEM UART.
The initial configuration targets one receiver and an NX7e+ using its 11 ms or 22 ms frame rate, at 115200 baud.
The driver is included in `px4_fmu-v6c_default` and is disabled until a serial port is selected.

:::warning
This implementation requires bench validation with the receiver before flight.
Firmware builds and software tests do not establish electrical compatibility, response timing, or transmitter display behaviour.
:::

## Wiring

Power off the flight controller before connecting the receiver.

| Receiver | Pixhawk 6C TELEM1 |
| --- | --- |
| Signal | TX, pin 2 |
| V+ | +5 V, pin 1 |
| GND | GND, pin 6 |
| N/C | Unconnected |

Leave TELEM1 RX, CTS, and RTS unconnected.
The signal wire carries both RC input and replies on the UART TX pin in half-duplex mode.
The driver uses an uninverted 3.3 V signal, not 5 V logic.
Bind using the receiver's button before performing RC calibration.

## Configuration

1. Release TELEM1 from other drivers.
   In the default Pixhawk 6C configuration, set `MAV_0_CONFIG` to **Disabled** (`0`).
   Check that no other MAVLink instance or serial driver selects TELEM1.
2. Set `RC_SRXL2_PRT_CFG` to **TELEM 1** (`101`).
3. Leave `RC_SRXL2_TEL_EN` enabled to send aircraft telemetry.
4. Reboot the flight controller.
5. Perform [RC calibration](../config/radio.md) and verify every stick and switch.

The driver selects 115200 baud and disables flow control; `SER_TEL1_BAUD` does not change SRXL2's baud rate.
It uses the same Spektrum channel scaling and throttle/roll/pitch remapping as PX4's DSM input.
The usual PX4 channel mapping, calibration, and RC-loss actions apply.
Up to 18 channels can be published.
Disabling `RC_SRXL2_TEL_EN` suppresses aircraft sensor payloads while retaining RC reception and required bus communication.
Set `RC_SRXL2_PRT_CFG` to **Disabled** and reboot before assigning the port to another driver.

## NX7e+ Telemetry

With the receiver bound and RF active, open **Function List > Telemetry > Auto Config** on the transmitter.
No transmitter scripts or additional applications are required.
Record the AirWare version used for testing; older firmware can differ in sensor availability and display behaviour.
AirWare 4.01 corrected a Flight Pack Capacity consumed-capacity display issue.

| Stock sensor | PX4 data |
| --- | --- |
| RPM/voltage/temperature | Battery instance 0 voltage and available temperature; RPM unavailable |
| Flight Pack Capacity | Battery instance 0 current and consumed capacity |
| GPS | Selected GNSS position, MSL altitude, ground speed, course, and satellites |
| Vario | Height above home and averaged climb/sink rates |
| Attitude/compass | Vehicle roll, pitch, and yaw; magnetic heading and magnetometer fields unavailable |
| TextGen | Flight mode, armed/failsafe state, and recent PX4 status messages |
| Flight Log/QoS | Receiver-populated link information |

Actual attitude display support must be checked on the transmitter's installed firmware.
Unavailable, invalid, or stale measurements use Spektrum's unavailable values.
GPS altitude is above mean sea level; vario altitude is relative to home.
Vario averaging windows need up to three seconds of valid history after startup or an estimator reset.
TextGen forwards the `mavlink_log` topic and does not display every structured event available in QGroundControl.

Telemetry is sent only in receiver-granted slots, after two control-frame intervals qualify as supported.
A busy UART, uncertain timing, or expired reply window causes the reply to be skipped.
Unsupported faster control timing pauses telemetry replies; RC reception continues.
Telemetry resumes automatically after two consecutive, fully observed control-frame intervals qualify as supported, including when the transmitter is switched on after the flight controller.
Missing packets, uncertain timing, or bus silence require fresh timing qualification before replies can resume.

## Diagnostics

Use the MAVLink console over USB:

```sh
srxl2_rc status
listener input_rc
listener rc_channels
```

For a manual test, first stop any service using the port, then run:

```sh
srxl2_rc start -d /dev/ttyS5
srxl2_rc status
srxl2_rc stop
```

Status reports received frames, parser errors, channel health, handshake state, and skipped replies.
Normal SRXL2 packets can update only a subset of channels; omitted channels retain their last received values, as specified by SRXL2 section 7.7.1.
The driver waits for all published channels to be initialised before reporting healthy RC.
Status includes the initialised channel mask and a separate mask of channels updated within 100 ms, in PX4 channel order.
A smaller recent-update mask is normal when the receiver holds auxiliary channels.
Handshake, telemetry, zero-channel fade packets, and updates entirely above the supported 18-channel range do not count as fresh RC samples.
Receiver failsafe is propagated immediately and clears the initialised normal channel values.
Bus silence resets receiver discovery and channel values after 50 ms; ongoing traffic without normal supported channel data causes RC loss after 100 ms.
The 100 ms control-packet timeout is a PX4 policy that must be checked with transmitter loss and recovery on the bench.

## Bench Acceptance

Keep propellers removed and verify:

- Both startup orders, controller restart while the receiver streams, receiver power cycling, and re-binding.
- Stick/switch mapping at 11 ms and 22 ms with all configured channels active.
- Transmitter loss, receiver failsafe, signal-wire disconnection, and PX4 RC-loss/recovery behaviour.
- Logic-analyser measurements of idle-high signalling, rise time, turnaround, and final stop-bit completion using the actual cable.
- RC and telemetry under CPU/IO load, including delayed processing that skips expired replies without transmitting late.
- One hour of operation with changing channels and no unexplained freezes or false healthy RC indications.
- Auto Config, sensor values/units, unavailable data, TextGen, and alarms on the recorded AirWare version.
- Stop/restart and reassignment of TELEM1 to MAVLink after SRXL2 is disabled.

Compare captures with ArduPilot using the same receiver and wiring.
Retain the firmware revision, captures, and flight-controller logs with the bench results.

## Implementation Limits

The initial implementation supports one receiver, one bus, and 115200 baud.
Software binding, 400 kbaud, 5.5 ms operation, receiver hubs/redundancy, VTX control, and forward programming are not supported.
Other boards require the timed STM32H7 serial transport and separate hardware validation.

## References

- [Spektrum SRXL2 specification](https://github.com/SpektrumRC/SRXL2/tree/6251dd14fbc95f262ecbc22e38ad39a8b907e411)
- [Spektrum telemetry definitions](https://github.com/SpektrumRC/SpektrumDocumentation/blob/4e179359e0916f347b214f7825640733f54d7996/Telemetry/spektrumTelemetrySensors.h)
- [NX7e+ manual](https://www.spektrumrc.com/on/demandware.static/-/Sites-horizon-master/default/dw0a5386f6/Manuals/SPMR7110_Manual_EN.pdf)
- [NX AirWare release notes](https://www.spektrumrc.com/on/demandware.static/-/Sites-horizon-master/default/dwedd0fb9e/Manuals/SPM_NX_AirWareChangeLog.pdf)
