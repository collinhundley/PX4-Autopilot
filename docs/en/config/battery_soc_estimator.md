# Calibrated Battery SOC Estimator

The calibrated state-of-charge (SOC) estimator combines coulomb counting with a measured nonlinear voltage profile.
It runs in the battery library used by analogue power modules, I2C power monitors and ESC battery measurements.
Sources that provide their own SOC retain that estimate.

`BAT1_SOC_MODE` defaults to `1` (shadow).
Shadow mode logs comparisons while the existing PX4 estimate continues to drive remaining capacity, remaining time and battery failsafes.
No calibrated curve is supplied: the voltage points default to zero, so the new estimate remains unavailable until a measured profile is configured.
Configure each battery independently using `BAT1`, `BAT2` or `BAT3` parameters.

## Battery Profile

Collect a controlled discharge from a confirmed full battery to the chosen usable-capacity endpoint, including motor-off relaxation intervals.
Measure total usable capacity in mAh, including charge consumed during rest intervals.
For every relaxed checkpoint, calculate `SOC = 1 - discharged_mah / measured_capacity_mah`.
Do not use a continuous loaded-voltage discharge curve as an open-circuit-voltage profile.
The profile applies to the measured chemistry, temperature range and battery condition; ageing requires revalidation of both capacity and curve.

Set `BAT1_CAPACITY` to measured usable capacity and `BAT1_N_CELLS` to the series cell count.
Set `BAT1_R_INTERNAL` to measured per-cell resistance, or retain the existing negative setting to use PX4's online resistance estimate.
Set `BAT1_SOC_V0` through `BAT1_SOC_V40` to relaxed cell voltages at 0%, 2.5%, …, 100% SOC.
All 41 points must be finite, within 2.5–5 V and strictly increasing.
These bounds validate configuration and do not define safe operating voltages for a particular chemistry.
Regenerate any earlier 21-point profile: `V20` now represents 50% SOC and `V40` represents 100% SOC.

The calibration helper converts relaxed checkpoints into a parameter script:

```sh
python3 Tools/battery_soc_profile.py relaxed.csv \
  --capacity-mah 4200 --cells 4 --resistance 0.010 --battery 1 > profile.params
```

The capacity, cell count and resistance above are examples; substitute measured values.
The CSV must contain `relaxed_cell_voltage_v` and `discharged_mah` columns with at least five relaxed checkpoints, including full (`discharged_mah = 0`) and usable-empty (`discharged_mah = measured_capacity_mah`) endpoints.
Use discharge relative to the full checkpoint, rather than an arbitrary lifetime counter.
The helper rejects missing endpoints, duplicate SOC checkpoints, non-finite values and non-monotonic curves.
Review the generated script before applying it through the PX4 shell.
It selects shadow mode and never enables the active estimator.

## Initialization And Charge Tracking

Startup follows the existing PX4 approach: filtered, resistance-compensated voltage supplies the starting SOC after the normal two-second battery sensor startup period.
The new estimator replaces linear interpolation between `BAT1_V_EMPTY` and `BAT1_V_CHARGED` with the calibrated 41-point lookup table.
This sets `battery_soc_status.initialized = true` and immediately enables the new estimate in active mode.
Startup has no low-current restriction or additional relaxation delay, and does not require a known vehicle state or disarming.
A partially charged battery starts from its interpolated profile value rather than an assumption of full charge.

The voltage-derived starting value is a defined SOC estimate; it is not an unknown state that must wait for a rest interval.
As with PX4's previous startup estimate, its accuracy depends on the profile, resistance compensation, temperature and recent load.
Subsequent voltage corrections require either disarmed relaxation or a semi-stable flight load.
Corrections remain gradual, including on the first qualified interval after a normal startup.
A qualified disarmed voltage at or above the full profile endpoint sets `full_charge_confirmed`.

Active mode never conceals a missing starting estimate or invalid profile/protection configuration behind the legacy estimator.
It reports unavailable remaining capacity (`remaining = -1`) until ready.
The battery arming check rejects unavailable or non-finite SOC for every connected battery, independently of `COM_ARM_BAT_MIN`.
The usual battery circuit breaker can still bypass battery checks; it must remain disabled for these checks to apply.
Shadow mode continues using the existing estimator and does not require a configured new profile to fly.

Coulomb counting then determines the change in SOC using measured discharge and configured usable capacity.
The independent `battery_status.discharged_mah` counter is never changed by a voltage correction.
Positive and negative voltage corrections are allowed, without a second clamp against `1 - discharged_mah / capacity`.

Arming and disarming do not reset SOC or measured discharge.
An observed disconnect resets both connection histories and requires initialization for the replacement battery.
A reboot starts a new history and does not assume a full battery.
The estimator cannot identify a pack swapped without an observed disconnect; restart the battery source or reboot after such a replacement.

`charge_history_valid` records whether measured charge history is complete since the latest voltage seed.
It is true on a normal startup with measured current, even before relaxation qualifies voltage correction.
Missing current, a reset discharge counter or a sample gap longer than five seconds clears that diagnostic flag, while the defined SOC estimate is retained.
Missing startup current does not prevent voltage-based initialization; unavailable current is omitted from resistance compensation.
A new qualified rest can establish a fresh voltage anchor after missing charge measurements.
Flight corrections remain gradual after missing measurements and do not clear `charge_history_valid` or replace the coulomb-counting baseline.
Missing or out-of-order samples cannot qualify relaxation or span a voltage-warning debounce interval.
An idle current overwrite is not accepted as a measured current for the new estimator.

## Voltage Corrections

The existing resistance compensation is retained:

```text
compensated_cell_voltage = pack_voltage / cell_count + current * per_cell_resistance
```

The calibrated profile uses the filtered compensated cell voltage.
Correction follows `alpha = 1 - exp(-dt / BAT_SOC_TC)`, giving consistent behaviour across update frequencies.
Both upward and downward corrections use the same time constant, without PX4's legacy weighting toward lower voltage-based SOC.

### During Flight

Flight corrections are enabled by default for the new estimator (`BAT_SOC_INFLT = 1`).
The overall estimator still defaults to shadow mode, so corrections affect flight-facing SOC only with `BAT1_SOC_MODE = 2`.
An existing saved `BAT_SOC_INFLT = 0` continues to disable them.

While armed, qualification uses measured electrical power (`pack_voltage * current`), with no absolute current limit.
The minimum and maximum power across the qualifying span must remain within 30% of their midpoint for 10 seconds by default.
For example, 170–230 W qualifies around a 200 W midpoint; 30% is the entire span, not ±30%.
Once qualified, corrections continue until the power span exceeds this limit.
Then qualification starts again at the new load, allowing short cruise spans between manoeuvres to contribute.
The span check also rejects a gradual ramp that exceeds the limit, even when individual steps are small.

Loaded voltage is less certain than relaxed voltage, even after resistance compensation.
The estimator therefore maps compensated voltage **minus and plus 20 mV/cell** through the lookup table to form a plausible SOC interval.
If coulomb-updated SOC falls within that interval, voltage causes no correction.
Otherwise, SOC moves slowly toward the nearest edge, rather than all the way toward the raw voltage estimate.
This reduces sensitivity to residual voltage sag, especially in flat parts of the voltage curve.

Flight voltage corrections are additionally capped at **2 SOC percentage points per minute of qualified time**, in either direction.
The 300-second time constant and rate cap both apply; whichever yields the smaller correction wins.
Measured current continues to deplete SOC independently and is never rate-limited.
A load step exceeding the power-span limit immediately stops voltage corrections, rejecting its brief voltage sag.

| Parameter        | Default     | Meaning                                                    |
| ---------------- | ----------- | ---------------------------------------------------------- |
| `BAT_SOC_INFLT`  | Enabled     | Allow qualified flight corrections                         |
| `BAT_SOC_F_SETT` | 10 s        | Semi-stable power duration before flight corrections start |
| `BAT_SOC_P_STAB` | 30%         | Full power span relative to the min/max midpoint           |
| `BAT_SOC_F_DV`   | 0.02 V/cell | Voltage uncertainty margin in either direction             |
| `BAT_SOC_F_RATE` | 2 pp/min    | Maximum voltage-driven SOC correction in either direction  |
| `BAT_SOC_TC`     | 300 s       | Voltage correction time constant, flight and disarmed      |

This can gradually reduce SOC overestimation when effective capacity under load or ageing makes the voltage trajectory fall behind nominal coulomb counting.
It does not learn or change `BAT1_CAPACITY`, and stable power does not prove that voltage is fully relaxed.
Persistent compensation error larger than the voltage margin can still bias SOC; the rate cap bounds the speed of that bias, not its eventual size.
Use flight logs to validate the margin, correction rate and profile for the battery and operating temperature.
Fast or large capacity losses may outpace these deliberately slow corrections; independent terminal-voltage protection remains necessary.

### While Disarmed

Disarmed corrections retain the relaxed-voltage qualification and use the full lookup estimate with the same time constant.
Only a qualified disarmed rest can confirm full charge or replace the voltage anchor after missing charge measurements.

| Parameter        | Default | Meaning                                                    |
| ---------------- | ------- | ---------------------------------------------------------- |
| `BAT_SOC_I_MAX`  | 2 A     | Maximum measured current for disarmed relaxation           |
| `BAT_SOC_I_STAB` | 0.5 A   | Full current span across the disarmed qualification period |
| `BAT_SOC_SETTLE` | 120 s   | Disarmed settling time; no startup or flight delay         |

The disarmed limits are starting settings, not measured relaxation requirements for every pack.
Idle current exceeding the limit disables disarmed correction without restricting startup or flight correction.
Arming or disarming starts a new qualification period but preserves SOC and measured discharge.
Stale vehicle status, missing measurements and sample gaps restart qualification in either state.
These gates never pause coulomb counting or deactivate an initialized estimator.
Parameter changes are deferred while armed; changing the profile or estimator settings while disarmed starts a new initialization.

## Independent Voltage Protection

In active mode (`BAT1_SOC_MODE = 2`), the worst of SOC and independent voltage warning severity is published in `battery_status.warning`.
Existing PX4 battery failsafe actions consume that field, including their existing severity latch while armed.
The voltage path never reduces reported SOC to manufacture a failsafe.
It remains effective when configured capacity is wrong, current is unavailable or the calibrated SOC profile is invalid.
External SOC sources retain their existing warnings.

Protection uses actual terminal pack voltage divided by configured cell count, without resistance compensation.
It cannot detect a weak individual cell without cell telemetry; pack-average voltage does not replace cell-level battery protection.

| Parameter      | Default     | Meaning                                               |
| -------------- | ----------- | ----------------------------------------------------- |
| `BAT1_V_LOW`   | 3.50 V/cell | Low warning threshold                                 |
| `BAT1_V_CRIT`  | 3.40 V/cell | Critical warning threshold                            |
| `BAT1_V_EMERG` | 3.30 V/cell | Emergency warning threshold                           |
| `BAT_V_TC`     | 1 s         | Terminal voltage filter time constant                 |
| `BAT_V_DELAY`  | 2 s         | Continuous debounce for severity changes and recovery |
| `BAT_V_HYST`   | 0.05 V/cell | Recovery margin above the active threshold            |

Thresholds must satisfy `V_LOW > V_CRIT > V_EMERG`.
Validate thresholds, filtering and debounce under representative flight loads, particularly VTOL transitions.
Defaults are starting settings, not validated limits for the user's battery.
Invalid protection configuration leaves active-mode SOC unavailable and prevents arming.
Shadow mode logs prospective voltage warnings without changing flight behaviour.

## Validation And Activation

The default logging profile records `battery_soc_status` at 5 Hz, with battery identity and sample timestamps.
It includes legacy, pure coulomb, profile-voltage and combined SOC; correction eligibility; initialization and charge-history validity; voltage/current/discharge; and separate SOC/voltage warnings.
Use `SDLOG_MODE = 1` when disarmed relaxation needs to be captured, and confirm the SD log records the topic.
Custom logging profiles must explicitly include `battery_soc_status`.

Compare shadow results with independently measured remaining capacity at checkpoints after full and partial starts, variable loads, flight, and post-flight recovery.
Include different update rates, incorrect capacity and reduced-capacity aged batteries.
The acceptance target is approximately ±5 percentage points at measured checkpoints, with emphasis on avoiding overestimation.
Synthetic functional tests verify estimator behaviour; they do not establish this accuracy on real batteries.

Verify that sustained low terminal voltage reaches the intended existing PX4 failsafe action even when SOC is high.
After profile and voltage-protection validation, select active mode while disarmed.
The estimate becomes active after normal battery sensor startup without waiting for relaxation.
Returning to shadow mode restores legacy output and retains comparison history when the profile is unchanged.
