# On-Screen Display (OSD)

An **On-Screen Display (OSD)** overlays flight telemetry — battery, altitude, GPS, RSSI, attitude, etc. — onto a pilot's video feed.
OSDs are commonly used in FPV and long-range flying so the pilot can see live flight data without looking away from the video.

PX4 supports three distinct OSD mechanisms, each targeting a different class of video system:

| Mechanism                               | Use case                                                                                                                               | Transport            | Runs on FC?                                                    |
| --------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- | -------------------- | -------------------------------------------------------------- |
| [MSP OSD](#msp-osd)                     | Digital FPV air units and video goggles that speak Betaflight MSP (e.g. DJI O3/O4, Walksnail, HDZero, Caddx Vista)                     | Serial, MSPv1        | Yes — [`msp_osd`](../modules/modules_driver.md#msp-osd) driver |
| [ATXXXX Analog OSD](#atxxxx-analog-osd) | Legacy analog video with an on-board MAX7456/ATXXXX overlay chip (e.g. OmnibusF4SD)                                                    | SPI to on-board chip | Yes — [`atxxxx`](../modules/modules_driver.md#atxxxx) driver   |
| [MAVLink OSD](#mavlink-osd)             | MAVLink-aware ground stations and displays that render their own OSD from telemetry (e.g. Yaapu on EdgeTX/OpenTX, Skydroid, mLRS HUDs) | Serial, MAVLink      | No — streams MAVLink; the display renders the OSD              |

Which one you use is determined by your video hardware, not by PX4 preference.
If you're unsure, start with your video system's documentation and match the OSD mechanism it expects.

## MSP OSD

**MSP (MultiWii Serial Protocol) OSD** is the mechanism used by digital FPV systems (DJI, Walksnail, HDZero) and by many digital goggles/air units to render telemetry over the pilot's video feed.
The `msp_osd` driver sends MSP DisplayPort drawing commands, not just MSP telemetry.
It emits MSPv1 message `182` with heartbeat, clear-screen, write-string and draw-screen subcommands at 10 Hz, alongside flight-controller identity, battery and arming telemetry.
DisplayPort support was added in [PX4 pull request #24695](https://github.com/PX4/PX4-Autopilot/pull/24695) and is included in this revision.

### Supported Displays

The current renderer uses a fixed layout with characters extending beyond the legacy 30-column canvas.
Use an HD canvas on displays that offer a canvas-size setting.
There is no PX4 parameter for arranging elements or selecting a different canvas size.

The displayed items are:

- Flight mode, arming state, heading and scrolling warnings.
- Average battery cell voltage.
- GPS latitude, longitude and satellite count.
- Distance to home.
- Altitude: GNSS altitude above mean sea level when a fix is available, otherwise height above the local estimator origin.
  This is not consistently height above home or terrain.
- RC link quality in the field labelled RSSI.
- Numeric pitch and roll.

Only the GPS latitude, longitude, satellite count, home distance, RSSI and altitude bits in [`OSD_SYMBOLS`](../advanced_config/parameter_reference.md#OSD_SYMBOLS) gate individual drawing commands in this renderer.
Setting the whole mask to zero suppresses drawing and the periodic battery/arming telemetry; it does not stop the driver or clear the last displayed frame.
The other listed items are drawn whenever the mask is nonzero.
The remaining bit labels and [`OSD_CH_HEIGHT`](../advanced_config/parameter_reference.md#OSD_CH_HEIGHT) come from the older telemetry-based layout and do not control the current DisplayPort layout.
Ground speed, a home-direction arrow, current, consumed mAh, total pack voltage and crosshairs are not drawn by this renderer.

### Hardware Setup

1. Connect the digital air unit's MSP / telemetry input to a free UART on the flight controller (TX → RX, RX → TX, GND → GND).
2. Power the air unit from its own BEC or a VTX power pad — use a supply within the air unit's specified voltage and current limits.
3. Note which PX4 serial device the UART maps to on your board (e.g. Pixhawk 6C `TELEM2` → `/dev/ttyS3`).
   See [Serial Port Mapping](../hardware/serial_port_mapping.md).

### Firmware Requirements

The `msp_osd` driver is included in the default build for most modern Pixhawk and FPV-oriented boards (e.g. `px4_fmu-v5x`, `px4_fmu-v6c`, `px4_fmu-v6x`, `ark_fpv`, `cuav_7-nano`, `micoair_h743*`).
If your board does not include it by default, enable it via [board config](../hardware/porting_guide_config.md#px4-menuconfig-setup):

```sh
make <board>_default boardconfig
# drivers → OSD → msp_osd
```

The corresponding board option is `CONFIG_DRIVERS_OSD_MSP_OSD=y`.
Rebuild the firmware after changing it.

### PX4 Configuration

1. Release the chosen UART from any other serial driver, including MAVLink and uXRCE-DDS.
2. Assign it to MSP OSD with [`MSP_OSD_CONFIG`](../advanced_config/parameter_reference.md#MSP_OSD_CONFIG).
3. Reboot.
   The driver sets 115200 baud, 8N1 and no hardware flow control internally; `SER_<PORT>_BAUD` does not override this rate.
4. Leave `OSD_SYMBOLS` at its default of `16383` initially.
   See the limitations above before changing individual bits.
5. Adjust [`OSD_LOG_LEVEL`](../advanced_config/parameter_reference.md#OSD_LOG_LEVEL), [`OSD_SCROLL_RATE`](../advanced_config/parameter_reference.md#OSD_SCROLL_RATE) and [`OSD_DWELL_TIME`](../advanced_config/parameter_reference.md#OSD_DWELL_TIME) for warning text and scrolling.
6. Set [`OSD_RC_STICK`](../advanced_config/parameter_reference.md#OSD_RC_STICK) to `0` unless VTX stick commands are needed and the RC channel mapping has been checked.
   The driver forwards raw channels in a fixed order while disarmed; it does not use PX4's RC mapping parameters.

### Pixhawk 6C With DJI O4 Air Unit And Goggles N3

Use TELEM2 for the OSD and leave [SRXL2 on TELEM1](./spektrum_srxl2.md).
On Pixhawk 6C, TELEM2 is UART5 (`/dev/ttyS3`), and TELEM1 is UART7 (`/dev/ttyS5`).

| Pixhawk 6C TELEM2       | DJI O4 Air Unit                                             |
| ----------------------- | ----------------------------------------------------------- |
| Pin 2, TX (3.3 V logic) | UART RX, white wire                                         |
| Pin 3, RX (3.3 V logic) | UART TX, grey wire                                          |
| Pin 6, GND              | Signal GND, brown wire; common with the power supply ground |

Leave TELEM2 pins 1 (5 V), 4 (CTS) and 5 (RTS) unconnected for this wiring.
Power the O4 Air Unit separately through its red power and black ground wires.
The non-Pro O4 Air Unit accepts 3.7–13.2 V; do not connect it directly to a 4S or higher battery.
Leave its yellow S.Bus wire unconnected when using SRXL2 for RC.
Verify connector orientation against [Holybro's Pixhawk 6C port pinout](https://docs.holybro.com/autopilot/pixhawk-6c/pixhawk-6c-ports) and the [DJI O4 Air Unit manual](https://dl.djicdn.com/downloads/DJI_O4_Air_Unit_Series/UM/DJI_O4_Air_Unit_Series_User_Manual_v1.0_en.pdf).

In *QGroundControl*, set the following and reboot:

| Parameter          | Setting                                   |
| ------------------ | ----------------------------------------- |
| `MSP_OSD_CONFIG`   | `102` (TELEM 2)                           |
| `OSD_SYMBOLS`      | `16383` (initial default)                 |
| `OSD_RC_STICK`     | `0` (disable VTX stick forwarding)        |
| `RC_SRXL2_PRT_CFG` | Keep `101` (TELEM 1)                      |
| `RC_SRXL2_TEL_EN`  | Keep the existing SRXL2 telemetry setting |

Before assigning MSP, check `MAV_0_CONFIG`, `MAV_1_CONFIG`, `MAV_2_CONFIG`, `UXRCE_DDS_CFG`, and any other serial-port assignments.
Disable or move only services assigned to TELEM2 (`102`).
Do not re-enable MAVLink on TELEM1 when that port is used by SRXL2.
Setting `SER_TEL2_BAUD=115200` is optional for clarity; the MSP driver selects 115200 itself.
Building the driver does not change saved port assignments: `MSP_OSD_CONFIG` defaults to Disabled.

Activate, update and link the O4 Air Unit and Goggles N3 using DJI's setup procedure.
Enable the goggles' OSD/Canvas Mode display and select an HD canvas if that setting is offered by the installed goggles firmware.
DJI's Betaflight CLI examples configure Betaflight, so use the PX4 parameters above instead.
PX4 generates the fixed layout itself; it cannot be edited with Betaflight Configurator.
DJI documents Canvas Mode, but does not list PX4 as a supported flight-controller firmware; verify this combination on the bench.

With propellers removed and cooling airflow over the powered air unit, check the following in the MAVLink Console:

```sh
msp_osd status
srxl2_rc status
```

MSP should report `/dev/ttyS3`, `initialized: 1`, increasing successful sends and no increasing failed sends.
SRXL2 should remain on `/dev/ttyS5`.
Verify that OSD values update in the goggles, that RC input still works, and that the arming indication follows the aircraft state.
Successful UART writes alone do not confirm that the goggles received or rendered the data.
If the OSD is absent, check the selected UART, crossed TX/RX, common ground and goggles OSD setting first.

### Worked Examples

- [Reptile Dragon 2 > msp_osd Module](../frames_plane/reptile_dragon_2.md#msp-osd-module) — end-to-end wiring and configuration for a Caddx Vista build.
- [Turbo Timber Evolution](../frames_plane/turbo_timber_evolution.md) — references the same setup pattern.

## MAVLink OSD

Some OSDs render their own overlay directly from the MAVLink telemetry stream — the flight controller simply streams MAVLink at a rate the display can parse.
PX4 exposes this via a dedicated MAVLink stream profile.

To use a MAVLink OSD:

1. Choose an unused MAVLink instance ([`MAV_X_CONFIG`](../peripherals/mavlink_peripherals.md#default_ports)) and assign it to the serial port connected to the display.
2. Configure the mode of the selected MAVLink instance with [`MAV_X_MODE`](./mavlink_peripherals.md#MAV_X_MODE) by setting it to **`OSD`**.
   The `OSD` mode uses a built-in rate table tuned for low-bandwidth OSD consumption.
3. Set the matching `SER_<PORT>_BAUD` to the baud rate the display expects.

The stream content is fixed (defined in `src/modules/mavlink/mavlink_main.cpp`) and cannot be customised from parameters.
See [MAVLink Peripherals (GCS/OSD/Gimbal/Camera/Companion)](./mavlink_peripherals.md) for the full MAVLink-side configuration.

## ATXXXX Analog OSD

The [`atxxxx`](../modules/modules_driver.md#atxxxx) driver targets boards with an on-board MAX7456 / ATXXXX chip that overlays characters onto an analog video stream (PAL or NTSC).
This was common on older F4-class FCs such as OmnibusF4SD and is largely superseded by digital systems.

No external wiring is required on boards that include the chip; to enable it, set [`OSD_ATXXXX_CFG`](../advanced_config/parameter_reference.md#OSD_ATXXXX_CFG) to `1` (NTSC) or `2` (PAL) and reboot.

## See Also

- [Parameter Reference > OSD](../advanced_config/parameter_reference.md#osd) — all OSD parameters.
- [MAVLink Peripherals (GCS/OSD/Gimbal/Camera/Companion)](./mavlink_peripherals.md) — MAVLink serial configuration.
- [Serial Port Configuration](./serial_configuration.md) — assigning modules to UARTs.
- [`msp_osd` module reference](../modules/modules_driver.md#msp-osd) — CLI usage and source.
