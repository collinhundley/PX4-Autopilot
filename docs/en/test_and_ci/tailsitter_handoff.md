# Tailsitter Controller Handoff

The tailsitter front transition transfers runtime controller state and matches the first fixed-wing (FW) motor command to the last multicopter (MC) command.
This is particularly useful for aircraft that use differential thrust on all three axes.
It does not change controller gains, the transition trajectory or persistent trim parameters.

## Readiness And Output Continuity

Commit `a081354933` retained MC collective for 50 ms because FW thrust was not necessarily available when the transition state changed.
Commit `f119b15ff18` subsequently set all three torque commands to zero during that interval.
The timer protected collective, but removed differential control and did not establish that a new FW command had actually arrived.

The replacement uses a request, acknowledgement and activation exchange.
The request identifies a particular transition and captures the last routed motor torque and collective, together with the MC controller's learned bias and rate target.
The FW controller acknowledges only after initialising from a fresh FW target and producing finite torque and thrust from a post-request angular-velocity sample.
The router retains the outgoing torque and collective until both publications associated with that acknowledgement have arrived.
An old acknowledgement cannot activate a later transition.

For altitude-controlled flight, the FW attitude target must also have been produced after the new TECS update.
The source attitude-setpoint timestamp is the generation time of that target, rather than the time the rate controller last republished a message containing it.
A separate FW rate-setpoint topic prevents an MC publication from satisfying this check.
Manually generated targets copy their thrust into the rate target together with the attitude and timestamp, so a fresh manual timestamp cannot acknowledge an old zero thrust value.

Outputs older than 100 ms are rejected.
This permits ordinary publication intervals and scheduler jitter without accepting indefinitely stale commands.
Missing publications retain the last valid motor command and trigger the existing quad-chute path after one second.
The MC callback remains available in FW tailsitter flight so this timeout can run even if FW publications stop.

## Three-Axis Bias And Transient Matching

The MC-to-FW vector transform is `[-z, y, x]`; its inverse is `[z, y, -x]`.
The transfer divides by the enabled differential-thrust scales before applying this transform.
It removes the FW scheduled trim and roll-to-yaw feedforward contribution, then divides by the actual controller output gain, including airspeed scaling and gain compression, to initialise the FW integral.
The integral is bounded by the existing per-axis limits.
Any representable bias that does not fit in the integral remains as non-integrating runtime trim.
Direct Acro yaw retains its transferred bias in this runtime trim because that mode deliberately disables yaw integration.

The transfer does not write `TRIM_ROLL`, `TRIM_PITCH`, `TRIM_YAW` or any gain parameter.
It replaces the previous transfer on each new handoff and clears transient state on cancellation, disarming or return to MC.
Allocator residuals are transformed into FW axes with the corresponding signs for anti-windup.
Controller clipping also inhibits further integration towards saturation.

Integral transfer alone cannot cancel changes in proportional, derivative or feedforward output.
An initial rate-target offset and a full torque-command mismatch therefore decay with a smoothstep over 0.5 seconds after acceptance.
Only these initial offsets decay; subsequent feedback remains active.
The router also removes a temporary motor-space residual when disabled or reduced differential authority prevents an exact FW representation.
Sustained torque beyond the configured FW authority cannot be preserved indefinitely.

## Collective And Demand Changes

Collective matching uses the outgoing MC motor command, including its throttle curve and battery scaling, rather than assuming that equal stick positions imply equal MC and FW thrust.
The FW command is matched after FW battery scaling.
A finite zero pilot demand is valid; an absent or non-finite publication is not treated as zero demand.

`VT_TS_THR_SLEW` sets the maximum collective reduction during MC → FW acquisition, in normalised command per second.
The default is `0.2`, corresponding to a 20 percentage-point reduction per second.
Increases use twice this rate to retain lift and airspeed margin.
There is no separate pilot-stick bypass; TECS underspeed protection bypasses the additional increase constraint.
Zero disables slew limiting while retaining readiness and first-output matching.

The limiter follows the current demand on every update and finishes when both the output value and the demand slew can be followed.
The demand's source timestamp distinguishes a new TECS or manual target from a repeated publication of the same target.
Crossing a rapidly moving target therefore cannot end acquisition prematurely.
Completion does not depend on an activation timer, and later commands cannot reactivate the limiter until a new MC → FW handoff.
It does not apply in hover, back-transition or ordinary FW control after acquisition.

The existing airspeed and altitude reference generators retain their original limits.
Airspeed slew starts from measured airspeed; mission commands before, during and after handoff update the live target normally.
No additional reference blend, altitude acceleration constraint or airspeed slew is introduced.
The older tailsitter waiting-for-TECS and one-second throttle blend is bypassed to avoid a competing ramp.
Other VTOL types retain their existing throttle path.

TECS is initialised from measured flight state and computes its normal energy-control demand behind the matched motor output.
Its initial height-rate control law matches the subsequent update, including the distinction between altitude and direct height-rate control.
The outgoing collective is not stored as a synthetic TECS integral bias.
The MC rate-controller biases are transferred separately as described above.

While the collective limiter is active, `tailsitter_handoff_status` returns the applied command, current demand and battery scale.
TECS inhibits integral accumulation towards the temporary limitation while allowing integration that unwinds it.
If `FW_THR_SLEW_MAX` is enabled, its existing bound starts from the same applied command during acquisition, so its ramp cannot advance independently ahead of the handoff limiter.
After acquisition, this tracking feedback is removed and normal TECS behaviour resumes.
The transient rate is the only new parameter; controller gains and persistent trim remain unchanged.

The default was selected using ordinary Dragonfly X-Plane handoffs with `0.1`, `0.2` and `0.3` command/s reductions.
In the comparable nose-up moment cases, acquisition took approximately 2.42, 1.01 and 0.59 seconds respectively.
The middle value removes the abrupt collective reduction without unnecessarily extending acquisition.
It is an airframe-based default, not a universal optimum; the time depends on the outgoing collective and the changing FW demand.
At the default rate, a full-scale reduction can take five seconds and a full-scale increase 2.5 seconds.
A rapid full-throttle request during the Dragonfly handoff reached full command after 1.51 seconds in the recorded test.

The slew acts on the motor-command convention, including the existing FW 2% motor-stop deadband.
A legitimate low target can therefore converge to zero collective; that is distinct from accepting an absent or uninitialised FW publication.
Controller continuity cannot supply sustained differential authority beyond the configured allocator limits.

## Validation Tools

`unit-tailsitter_handoff_test`, `functional-TailsitterHandoff`, the existing rate-control suite and `functional-TECS` cover frame transforms, nonzero bias on every axis, existing trim, unequal differential scales, gain scaling, integrator limits, anti-windup, P/D/feedforward mismatch, arbitrary throttle, delayed and invalid publications, old acknowledgements, repeated and aborted transitions, and unchanged hover routing.
The module test invokes the actual FW controller and tailsitter router through uORB.

`test/tailsitter_handoff_xplane.py` runs guarded ordinary transitions in the original Dragonfly using the existing runway-reset and calibrated CFD-moment helpers.
It never copies MC integral into trim parameters or stages a matched-thrust handoff.
Each case saves parameters, firmware identity, environment settings, telemetry, console output, ULog and a flight result.
The separate `test/analyze_tailsitter_handoff.py` checks torque, collective and all four motor commands at both the transition request and FW acceptance, and reports the following transient separately.

`test/tailsitter_handoff_forces.cpp` is an X-Plane test plugin, not part of PX4 firmware.
It adds signed roll, pitch and yaw moments to the simulator's force accumulators without cancelling native aerodynamics or changing control gains.
It starts disabled, limits each moment to ±2 N m, and expires each request after at most 180 seconds.

The simulator evidence must distinguish a successful handoff from a flight that never reached FW, or a later flight-envelope or back-transition failure.
The calibrated CFD moment law is a local approximation and does not establish a full nonlinear aerodynamic model or real-aircraft validation.
