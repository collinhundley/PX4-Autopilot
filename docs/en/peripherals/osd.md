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
It emits MSPv1 message `182` with heartbeat, clear-screen, write-string and draw-screen subcommands at a nominal 10 Hz, alongside flight-controller identity, battery and arming telemetry.
DisplayPort support was added in [PX4 pull request #24695](https://github.com/PX4/PX4-Autopilot/pull/24695) and is included in this revision.

### Displayed Items

The DisplayPort renderer provides the following fields. Each field has its own `OSD_SYMBOLS` bit.
Numeric fields show `--` when their source is unavailable, invalid or stale, except airspeed and ground speed, which show `0.0`.
This speed fallback affects the display only; it does not change estimator validity or filter small valid speeds.
When the home-direction item is enabled, it always renders an arrow.
Unknown, invalid or stale home bearing or camera heading displays a straight-ahead arrow until a valid direction becomes available.
This fallback affects the display only; it does not declare the home bearing valid.

| Item | Value displayed |
| ---- | --------------- |
| Battery capacity used | Consumed mAh from the primary battery (`battery_status` instance 0), switching to Ah at 10,000 mAh. |
| Current draw | Current in amperes from the same battery. |
| Compensated pack voltage | Filtered load-compensated voltage used by PX4's voltage-based state-of-charge calculation (`battery_status.voltage_v_compensated`). |
| Compensated average cell voltage | The same compensated pack estimate divided by cell count. |
| Battery remaining | Existing battery state-of-charge estimate (`battery_status.remaining`) as a percentage. |
| Flight time | Time since Commander's takeoff timestamp, frozen at landing or disarm. The last duration remains until the next takeoff; this is not time since arming or boot. |
| Flight mode | User-visible navigation mode, with `MC`, `FW`, `>MC` or `>FW` for VTOL state and transitions. |
| Airspeed | Validated calibrated airspeed (CAS) from `airspeed_validated.calibrated_airspeed_m_s`. `*` after the airspeed glyph (or `AS*` in Betaflight mode) identifies an estimated source, such as ground speed minus wind or synthetic airspeed. Ground speed is not silently substituted. |
| Ground speed | Magnitude of fused horizontal velocity. |
| Altitude | Height relative to home using valid local vertical position and the home altitude reference. There is no fallback to GNSS altitude above sea level. |
| Home direction and distance | Horizontal distance from fused global position, plus an arrow relative to the camera's horizontal heading. When direction is unavailable at home or the camera points nearly vertically, the arrow points straight ahead. |
| Artificial horizon | Earth horizon projected into the fixed camera frame, using vehicle attitude and camera mounting/FOV parameters. |
| Messages | User-facing MAVLink log messages, including autotune progress at the default INFO threshold. |
| Throttle percentage | Commanded thrust magnitude, using the larger valid, fresh VTOL thrust instance during transition. Axes marked NaN mean stopped motors and contribute zero, so an idle pusher does not hide hover thrust. This is operator feedback, not RC-stick position or measured motor output. Disarmed throttle is zero. |

Additional fields include total pack voltage, average cell voltage, electrical power, GNSS latitude/longitude and satellite count, RC RSSI percentage, vertical speed, numeric pitch/roll, crosshairs, horizon sidebars and armed/disarmed/failsafe status.
Arming state appears immediately after flight mode as `ARM` or `DIS`; an appended `!` indicates failsafe. The former PX4 label has been removed, with bit 0 reserved to preserve saved masks.
The RSSI field uses the receiver's RSSI percentage, not its separate link-quality value.
The `ESC_TMP` bit remains reserved and has no renderer.

Measured pack/cell voltages retain voltage sag; source calibration and filtering may still apply.
The compensated readings use the exact cell-voltage filter in the battery library, including the configured `BATn_R_INTERNAL` override or estimated resistance when selected.
This is distinct from the diagnostic `ocv_estimate_filtered`, which always uses estimated resistance.
No second compensation or filter is applied by the OSD.
Compensated voltage is unavailable until the battery estimator initialises; battery sources that do not publish it show `--`, without falling back to measured voltage or diagnostic OCV.
Cell voltage is an average, not an individual-cell measurement.

The existing PX4 remaining-charge estimator initialises from voltage, including a partially charged battery at startup.
When capacity is configured, it then fuses voltage with integrated current consumption; otherwise it uses the voltage estimate.
Battery warnings use this state of charge, while smart/external battery sources may supply their own estimate.
The OSD displays that published estimate directly; it does not calculate `1 - consumed_mAh / capacity` or assume an initially full battery.
An invalid, stale or disconnected reading is `--%`; a valid empty battery is `0%`.

Messages use a 30-character scrolling window and a bounded four-message queue.
More severe messages take priority, and `OSD_MSG_TIME` limits their age from publication, including time spent waiting in the queue.
The default is 10 seconds; reading the same message again does not extend its life.
`OSD_LOG_LEVEL=6` includes INFO messages such as autotune progress; `8` disables messages.
`OSD_SCROLL_RATE` controls each scroll step and `OSD_DWELL_TIME` controls the pause at each end.
Only messages published through `mavlink_log` are shown; not every PX4 event has a corresponding text message.

### Canvas, Units And Camera

`OSD_UNITS` defaults to `1` (imperial): feet, miles per hour and feet per second.
Set it to `0` for meters, meters per second and meters per second vertical speed.
Current, capacity, voltage, power and angles keep their electrical/angular units.

`OSD_CANVAS` selects the layout:

| Value | Canvas |
| ----- | ------ |
| `0` (default) | Accept the air unit's MSP canvas announcement; use 53 × 20 until a valid announcement arrives. |
| `1` | 30 × 16 |
| `2` | 50 × 18 |
| `3` | 60 × 22 |
| `4` | 53 × 20 (DJI O4 / Goggles N3) |

The default item coordinates use the 53 × 20 DJI layout: flight mode and arming state at top left, home information at top centre, and flight time at top right.
These header items use row 1, leaving row 0 blank to avoid top-edge clipping; messages use row 3.
DisplayPort positions use whole character cells, so fractional-row adjustments are unavailable.
Airspeed sits to the left of the centred horizon, altitude to the right, and ground speed below airspeed with two blank rows between them.
INAV mode prefixes airspeed with its dedicated airspeed icon; Betaflight mode uses `AS`.
Throttle sits in the bottom-left corner with its throttle glyph.
Battery percentage, consumed capacity and current occupy fixed slots on row 18, in that order.
A battery glyph immediately precedes the percentage, with its fill level following the reported charge; unavailable charge shows an empty outline with `--%`.
Compensated pack and average cell voltage sit below this group as `C 16.0V/4.00V` on row 19.
Measured voltages remain selectable on row 17.
Satellite count and RC RSSI sit at bottom right on rows 18 and 19; optional watts sit above throttle on row 17.
Pitch, roll, vertical speed, latitude and longitude are disabled in the default mask but remain selectable.
Coordinates stay fixed when readings, enabled items or canvas dimensions change.
For a different canvas, adjust the position parameters to fit; fields outside the canvas are clipped.
For 30 × 16, 50 × 18 and 60 × 22, the driver sends the matching DisplayPort resolution option.
The 53 × 20 canvas and other negotiated dimensions are respected without sending a conflicting standard resolution option.
Use `OSD_POS_CROSS_X` and `OSD_POS_CROSS_Y` to position the crosshair centre.

For a fixed camera aligned with the aircraft's body X axis, leave `OSD_CAM_PITCH=0`.
A positive mounting pitch points the camera upward relative to body X.
The renderer uses this fixed mounting transform for both the horizon and the home arrow, without switching frames when VTOL flight mode changes.
If the camera points level in forward flight but is not aligned with body X, set its actual mounting angle instead.
`OSD_CAM_VFOV` defaults to 60 degrees; tune it to the effective vertical field of view of the selected video mode and crop.
The projection assumes a 16:9 video image.

`OSD_FONT=0` (default) uses the Betaflight glyph map. `OSD_FONT=1` uses the INAV map supported by DJI Goggles N3, including font page 1 for the horizon, arrows and reticle.
This corresponds to the INAV font-table option documented for N3 in [ArduPilot's DisplayPort setup](https://ardupilot.org/plane/docs/common-displayport.html#configuration).
The driver also sends the matching `BTFL` or `INAV` MSP compatibility identifier. This selects protocol compatibility; the firmware remains PX4.
Units use glyphs for mph, feet/meters, mAh and amps, along with a home icon and directional arrow.
INAV mode adds the combined altitude/unit glyph and Ah glyph. Betaflight mode uses an `AH` text fallback because its standard font has no Ah glyph.
The font maps are documented in the [Betaflight glyph reference](https://betaflight.com/docs/development/OSD-Glyps) and [INAV symbol definitions](https://github.com/iNavFlight/inav/blob/master/src/main/drivers/osd_symbols.h).
Select a matching font in the display when available, and verify glyphs and horizon orientation on the bench.
Static horizon sidebars are enabled by default, with airspeed beside the left bar and altitude beside the right bar; scrolling speed/altitude tapes are not implemented.
A received canvas announcement or successful UART write does not confirm that a display renders these glyphs correctly.

### Item Positions

Every implemented display item has separate X and Y parameters in the **OSD Layout** group in *QGroundControl*.
All position parameters use the `OSD_POS_` prefix, followed by the item identifier and `_X` or `_Y`.
`SBAR` identifies the sidebar pair and `BATPCT` identifies battery percentage, keeping names within the 16-character parameter limit.
Positions apply while the driver is running, normally within about one second; no restart or firmware rebuild is needed after installing firmware with these parameters.
`OSD_SYMBOLS` continues to control which items are enabled.

- X is the column, increasing to the right; Y is the row, increasing downward.
- Coordinates start at zero in the top-left corner. For 53 × 20, use X `0`–`52` and Y `0`–`19`.
- Each parameter defaults to its explicit coordinate in the 53 × 20 layout shown below. There is no automatic position value.
- X anchors the first character or icon. The horizon, sidebars and crosshair use their centre instead.
- Parameter limits extend to X `59` and Y `21` for larger canvases. Content outside the active canvas is clipped; overlapping fields are not automatically rearranged.

| Item | X Parameter | Y Parameter | Default X | Default Y |
| ---- | ----------- | ----------- | --------- | --------- |
| Arming state | `OSD_POS_ARM_X` | `OSD_POS_ARM_Y` | 18 | 1 |
| Latitude | `OSD_POS_LAT_X` | `OSD_POS_LAT_Y` | 1 | 15 |
| Longitude | `OSD_POS_LON_X` | `OSD_POS_LON_Y` | 28 | 15 |
| Satellite count | `OSD_POS_SATS_X` | `OSD_POS_SATS_Y` | 46 | 18 |
| Ground speed | `OSD_POS_GSPD_X` | `OSD_POS_GSPD_Y` | 11 | 9 |
| Home icon and distance | `OSD_POS_HDIST_X` | `OSD_POS_HDIST_Y` | 22 | 1 |
| Home direction arrow | `OSD_POS_HDIR_X` | `OSD_POS_HDIR_Y` | 23 | 1 |
| Measured pack voltage | `OSD_POS_VOLT_X` | `OSD_POS_VOLT_Y` | 21 | 17 |
| Current | `OSD_POS_CURR_X` | `OSD_POS_CURR_Y` | 32 | 18 |
| Consumed capacity | `OSD_POS_MAH_X` | `OSD_POS_MAH_Y` | 24 | 18 |
| RC RSSI | `OSD_POS_RSSI_X` | `OSD_POS_RSSI_Y` | 46 | 19 |
| Altitude above home | `OSD_POS_ALT_X` | `OSD_POS_ALT_Y` | 34 | 6 |
| Vertical speed | `OSD_POS_VSPD_X` | `OSD_POS_VSPD_Y` | 41 | 11 |
| Flight mode | `OSD_POS_MODE_X` | `OSD_POS_MODE_Y` | 1 | 1 |
| Pitch angle | `OSD_POS_PITCH_X` | `OSD_POS_PITCH_Y` | 1 | 14 |
| Roll angle | `OSD_POS_ROLL_X` | `OSD_POS_ROLL_Y` | 39 | 14 |
| Crosshair centre | `OSD_POS_CROSS_X` | `OSD_POS_CROSS_Y` | 26 | 7 |
| Measured average cell voltage | `OSD_POS_CELL_X` | `OSD_POS_CELL_Y` | 27 | 17 |
| Sidebar pair centre | `OSD_POS_SBAR_X` | `OSD_POS_SBAR_Y` | 26 | 7 |
| Electrical power | `OSD_POS_POWER_X` | `OSD_POS_POWER_Y` | 1 | 17 |
| Flight time | `OSD_POS_TIME_X` | `OSD_POS_TIME_Y` | 42 | 1 |
| Airspeed | `OSD_POS_ASPD_X` | `OSD_POS_ASPD_Y` | 11 | 6 |
| Artificial horizon centre | `OSD_POS_HORIZ_X` | `OSD_POS_HORIZ_Y` | 26 | 7 |
| Messages | `OSD_POS_MSG_X` | `OSD_POS_MSG_Y` | 1 | 3 |
| Throttle | `OSD_POS_THR_X` | `OSD_POS_THR_Y` | 1 | 19 |
| Compensated pack voltage | `OSD_POS_CVOLT_X` | `OSD_POS_CVOLT_Y` | 20 | 19 |
| Compensated average cell voltage | `OSD_POS_CCELL_X` | `OSD_POS_CCELL_Y` | 28 | 19 |
| Battery percentage and icon | `OSD_POS_BATPCT_X` | `OSD_POS_BATPCT_Y` | 16 | 18 |

The sidebar pair moves as one item; speed and altitude positions remain independently adjustable.
Arming state, home arrow and battery fields each keep their own coordinates when other items move or are disabled.
Pack and average cell voltage retain their `V/V` presentation when both are enabled on the same row and the cell field leaves enough space for the pack field.
The separator occupies the cell field's preceding column; each number starts at its own X coordinate.
Otherwise the two fields render separately, each with its own `V` and, for compensated values, `C` marker.

For example, these MAVLink Console commands select 53 × 20 and place airspeed and ground speed at column 2, with two blank rows between them:

```sh
param set OSD_CANVAS 4
param set OSD_POS_ASPD_X 2
param set OSD_POS_ASPD_Y 6
param set OSD_POS_GSPD_X 2
param set OSD_POS_GSPD_Y 9
```

Reset an item's position parameters in *QGroundControl* to restore the coordinates in the table.
For example, `param reset OSD_POS_ASPD_X` restores column 11 and `param reset OSD_POS_ASPD_Y` restores row 6.
Earlier saved negative position values are reset to the new fixed defaults when the OSD runs; valid custom coordinates are preserved.

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

For the Pixhawk 6C aircraft configuration in this branch, build:

```sh
make px4_fmu-v6c_osd
```

The `osd` configuration inherits the default board configuration and excludes the uXRCE-DDS client, onboard SIH simulator and Septentrio GNSS driver to make firmware space available.
It retains the default flight-control features, including both autotuners, SRXL2, logging and MSP OSD.
The existing `px4_fmu-v6c_default` configuration is unchanged by these exclusions.
Desktop SITL remains available separately; removing the onboard SIH simulator does not prevent desktop simulation.
Use the repository's normal build environment for your host OS. Building does not flash the controller or change saved parameters.

### PX4 Configuration

1. Release the chosen UART from any other serial driver, including MAVLink and uXRCE-DDS if included in the firmware.
2. Assign it to MSP OSD with [`MSP_OSD_CONFIG`](../advanced_config/parameter_reference.md#MSP_OSD_CONFIG).
3. Reboot.
   The driver sets 115200 baud, 8N1 and no hardware flow control internally; `SER_<PORT>_BAUD` does not override this rate.
4. Select the desired `OSD_SYMBOLS` fields. The default is `1070882546`, which includes the flight fields, horizon sidebars, compensated pack/cell voltage and remaining battery percentage. Measured voltages, power, pitch, roll, vertical speed, latitude and longitude remain optional.
5. Set `OSD_UNITS`, `OSD_FONT`, `OSD_CANVAS`, `OSD_CAM_PITCH` and `OSD_CAM_VFOV` for the display and camera.
6. Leave `OSD_LOG_LEVEL=6` to include autotune progress, and adjust `OSD_MSG_TIME`, `OSD_SCROLL_RATE` and `OSD_DWELL_TIME` as desired.
7. Leave [`OSD_RC_STICK`](../advanced_config/parameter_reference.md#OSD_RC_STICK) at its default `0` unless VTX stick commands are needed and the RC channel mapping has been checked.
   When enabled, the driver forwards fresh raw channels in a fixed order while disarmed; it does not use PX4's RC mapping parameters.

Existing `OSD_SYMBOLS` bit numbers are retained. The new items use these bits:

| Bit | Item |
| --- | ---- |
| 22 | Flight time |
| 23 | Airspeed |
| 24 | Artificial horizon |
| 25 | Messages |
| 26 | Throttle percentage |
| 27 | `BATT_COMP_VOLTAGE` — compensated pack voltage |
| 28 | `BATT_CELL_COMP_VOLTAGE` — compensated average cell voltage |
| 29 | `BATT_PERC` — remaining battery percentage |

Flight mode uses the existing bit 14, and other existing bit labels now control their corresponding rendered fields.
Saved parameter values are not automatically overwritten on upgrade.
For example, an existing mask of `16383` does not enable flight mode or any of bits 22–29.
To adopt the new default layout, explicitly set `OSD_SYMBOLS=1070882546`; otherwise select the desired bits individually.
The three new battery items are enabled in this default mask and can be selected independently.
Use `OSD_SYMBOLS=1071407090` to also show measured voltages above the battery group.
The prior mask `131882994` shows measured voltages without compensated voltages or percentage; `130834418` also omits sidebars.
Prior masks `131039230` and `131039231` retain pitch, roll, vertical speed, latitude and longitude; the former PX4-label bit is ignored.
Also check `OSD_LOG_LEVEL`: an existing saved value of `3` continues to hide INFO/WARNING messages until changed.

Setting `OSD_SYMBOLS=0` clears and releases the overlay. The running driver continues its normal MSP flight-controller identity, battery and arming telemetry independently of the display mask.

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
| `OSD_SYMBOLS`      | `1070882546` (compensated voltages, percentage and sidebars) |
| `OSD_UNITS`        | `1` (imperial; choose `0` for metric)     |
| `OSD_FONT`         | `1` (INAV glyph map for Goggles N3)      |
| `OSD_CANVAS`       | `4` (fixed 53 × 20; `0` also falls back to 53 × 20) |
| `OSD_LOG_LEVEL`    | `6` (include INFO/autotune progress)      |
| `OSD_CAM_PITCH`    | `0` if the camera is aligned with body X |
| `OSD_CAM_VFOV`     | `60` initially; match the video mode     |
| `OSD_RC_STICK`     | `0` (disable VTX stick forwarding)        |
| `RC_SRXL2_PRT_CFG` | Keep `101` (TELEM 1)                      |
| `RC_SRXL2_TEL_EN`  | Keep the existing SRXL2 telemetry setting |

Before assigning MSP, check `MAV_0_CONFIG`, `MAV_1_CONFIG`, `MAV_2_CONFIG`, `UXRCE_DDS_CFG` if present, and any other serial-port assignments.
Disable or move only services assigned to TELEM2 (`102`).
Do not re-enable MAVLink on TELEM1 when that port is used by SRXL2.
Setting `SER_TEL2_BAUD=115200` is optional for clarity; the MSP driver selects 115200 itself.
Building the driver does not change saved port assignments: `MSP_OSD_CONFIG` defaults to Disabled.

Activate, update and link the O4 Air Unit and Goggles N3 using DJI's setup procedure.
Enable the goggles' OSD/Canvas Mode display and select a matching HD canvas/font if those settings are offered by the installed goggles firmware.
DJI's Betaflight CLI examples configure Betaflight, so use the PX4 parameters above instead.
PX4 generates the layout itself; edit its numerical X/Y parameters in *QGroundControl*, not Betaflight Configurator.
DJI documents Canvas Mode, but does not list PX4 as a supported flight-controller firmware; verify this combination on the bench.

With propellers removed and cooling airflow over the powered air unit, check the following in the MAVLink Console:

```sh
msp_osd status
srxl2_rc status
```

MSP should report `/dev/ttyS3`, `initialized: 1`, increasing successful sends, no increasing failed sends and no sustained transmit backlog.
The send counters measure packets accepted into the transmit queue; inspect pending bytes and the displayed image as well.
SRXL2 should remain on `/dev/ttyS5`.
Verify that OSD values update in the goggles, that RC input still works, and that the arming indication follows the aircraft state.
Check horizon direction while pitching and rolling the aircraft, the home arrow, imperial/metric labels, autotune text visibility and recovery after restarting the air unit.
Tune camera mounting/FOV parameters against the actual video before relying on the horizon.
Successful UART writes alone do not confirm that the goggles received or rendered the data.
If the OSD is absent, check the selected UART, crossed TX/RX, common ground and goggles OSD setting first.

### Automated Checks

In the normal PX4 Linux build environment, the OSD tests can be built and run with:

```sh
make px4_sitl_test
cmake --build build/px4_sitl_test --target unit-MspV1 unit-DisplayPort functional-OsdTelemetry functional-Battery
ctest --test-dir build/px4_sitl_test --output-on-failure -R '^(unit-(MspV1|DisplayPort)|functional-(OsdTelemetry|Battery))$'
python3 src/drivers/osd/msp_osd/test_displayport.py build/px4_sitl_test
```

The C++ tests exercise telemetry validity, timer/message state, rendering, per-item placement and clipping, serial packet handling, the published compensation filter and partially charged battery initialization.
The PTY test runs the SITL driver and inspects its emitted MSP/DisplayPort packets, including live position changes and 53 × 20 canvas selection.
These checks do not replace testing the O4/N3 font, canvas and video projection on actual hardware.

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
