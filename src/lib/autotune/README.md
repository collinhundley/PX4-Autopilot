# Autotune status ownership and VTOL regression checks

`autotune::Session` serializes ownership of the single
`autotune_attitude_control_status` topic. The library retains one uORB
advertisement for the lifetime of the process. Individual MC/FW modules never
advertise or unadvertise that topic. Stopping either module therefore cannot
invalidate a surviving publisher. This does not change generic uORB semantics.

Only an eligible idle tuner can acquire the channel. A mutex serializes acquire,
publish, and release, and an accepted command timestamp cannot be reused by a
late subscriber. The owner publishes zero excitation before releasing after an
abort or shutdown; later writes by that former owner are ignored. Terminal
results are normally retained for two seconds before publishing IDLE and
releasing. A flight-regime change releases immediately after terminating the
old attempt, permitting a new explicit command for the other tuner.

Status includes the owning `vehicle_type` and constant `timestamp_start` for
the attempt. Both attitude controllers require their own type and reject
excitation during transitions. The MAVLink receiver uses these fields and
publication freshness to correlate progress; old terminal results cannot
complete a new request. The status message is internal and unversioned; the
MAVLink command and progress interface are unchanged.

Both modules check current vehicle status before commands, state-machine
entries, and gain application. Their 100 ms watchdog also runs without torque
input. Unknown, stale (over two seconds), transitioning, or wrong-type vehicle
status aborts identification, testing, and pending disarm application. Lost
controller/gyro input aborts an active experiment after one second. Pending
disarm and terminal states publish zero-excitation heartbeats without needing
controller input. Aborting a trial restores backed-up gains once; completed
trials retain their accepted gains. No identification thresholds, PID design,
or excitation defaults are changed.

FW AUX is edge-triggered. An ON observed while ineligible is consumed, so the
switch must return OFF before a later ON can start a tune. QGC and mission
commands are consumed once; polling cannot latch a start in the inactive tuner.

## MAVLink startup and retries

A valid first command is dispatched even when the status topic has never been
published. Its startup handshake may report initialization for at most three
seconds. Once running, status older than one second fails the request. A fresh
active mission/AUX attempt may also be adopted without dispatching another
start. Transitions and unavailable vehicle status are rejected before dispatch.

QGC sends identical commands once per second and supplies no request identity.
After a missing/stopped-module failure, continuous polls remain failed. A pause
longer than three seconds permits a new explicit attempt. Observing the actual
terminal-to-IDLE state also permits the next tune after the normal cooldown.

## Deterministic tests

In a supported Linux PX4 development environment, from the repository root:

```sh
cmake -S . -B build/px4_sitl_test -G Ninja -DCONFIG=px4_sitl_test -DBUILD_TESTING=ON -DTESTFILTER=Autotune
cmake --build build/px4_sitl_test --target functional-AutotuneModule unit-MavlinkAutotune -j6
ctest --test-dir build/px4_sitl_test --output-on-failure -R '^(functional-AutotuneModule|unit-MavlinkAutotune)$'
```

The functional fixture calls the real MC/FW autotuners and attitude controllers
with real uORB publications/subscriptions. It drives scheduling synchronously
and seeds identification/test states where testing orchestration or rollback
does not require convergence. It covers command selection, AUX rejection,
publication lifetime in both stop orders, delayed commands, actual excitation
consumption, gain rollback, and repeated attempts. The separate MAVLink suite
exercises startup/progress/timeout correlation with deterministic timestamps.

These tests check routing and lifecycle behavior; they do not demonstrate
convergence or validate flight gains on an aircraft.
