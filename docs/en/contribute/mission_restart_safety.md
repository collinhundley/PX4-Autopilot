# Mission Restart And VTOL Transition Safety

This note records local safety changes made on 3 October 2026 following an unintended ground-to-fixed-wing transition on 2 October 2026.
Preserve the behaviour and regression coverage described here when rebasing or merging upstream.
An upstream implementation may replace these changes only after equivalent behaviour has been verified; retaining a mission cursor must not silently restore permission to execute it on a new flight.

## Incident And Cause

The intended mission began with a VTOL takeoff to 100 ft, followed by fixed-wing flight at 45 mph.
The pilot armed in Position mode and selected Mission with the RC switch, without a connected ground station.
The vehicle skipped takeoff, changed to fixed-wing control while landed, and accelerated to approximately 60 mph.

The investigation used flight logs from firmware `dc93cfa5daf7397eeec31e07bae039811e7989e7`:

- `good-flight.ulg` completed the preceding successful mission.
- The intervening `bad-aspd-1.ulg` attempted the revised mission, aborted its forward transition after a timeout, landed, and disarmed with mission item 1 still selected.
- `mission-bad.ulg` restored item 1 on the next boot, before arming.
  Selecting Mission issued the forward-transition command approximately 19 ms later, and the transition completed in approximately 5 ms while the land detector still reported landed.

Indices are zero-based: item 0 was takeoff and item 1 was the forward-transition command.
The retained cursor therefore bypassed takeoff.
Three behaviours combined to cause the incident:

1. Mission progress was restored from persistent storage and incomplete missions did not automatically restart after disarming.
2. `VtolType::isFrontTransitionCompleted()` treated either disarmed or landed as sufficient to finish a forward transition immediately.
3. The mission's initial climb handling covered position items, allowing a resumed transition command to execute before that handling.

The mission still requested 45 mph.
Its stored speed command was not the cause of the skipped takeoff or immediate control handover.
The pre-arm multicopter-mode warning had cleared by arming; clearing it did not reset mission progress.

## Mission Restart And Deliberate Resume

The first Mission activation after boot or a disarm/arm cycle starts execution at item 0.
Changing away from Mission and back during the same armed flight continues progress.
This preserves in-flight pause/resume while preventing an old checkpoint from deciding how a new flight begins.

Persistent mission storage and its format remain available for inspecting progress and planning an intentional resume.
Resetting execution on activation does not erase the saved checkpoint at boot; subsequent mission progress can update it through the existing storage path.
The checkpoint is not an immutable mission history.

An explicit start at a later item uses the existing `MAV_CMD_MISSION_START` command with the zero-based index in parameter 1.
The vehicle must already be armed and airborne.
The pilot must first establish a suitable position for the selected segment; the new guard checks armed/landed state, not route clearance or geographic suitability.
Selecting a waypoint alone does not override the first-activation restart policy.
A resume plan rebuilt by a ground station can instead begin at its own item 0 with a takeoff sequence.

The implementation deliberately uses existing mission state and commands, without adding a parameter, storage format, or separate resume protocol:

- `MissionBase::checkMissionRestart()` resets the execution cursor, jump counters, cached commands, speed override, ROI, and heading alignment for a fresh Mission flight.
  The additional restart rule applies to Mission and preserves the existing policy for RTL's `MissionBase` instances.
- `Mission::set_current_mission_index()` validates an explicit request even when it selects the already-stored index.
  Successful selection suppresses the automatic restart only for the current arming cycle.
  Item-zero starts reset loop state before resolving a possible `DO_JUMP` at the beginning of the mission.
- Commander rejects a nonzero ground start before changing flight mode or arming.
  Navigator independently checks the state before accepting the selected index.
- Resuming a forward-transition item in multicopter flight goes through the existing initial climb handling before issuing the transition.
  Being just above the ground is not sufficient to skip that required climb.

## VTOL Ground Transition Guards

Armed, landed forward-transition requests are rejected on both input paths: RC action requests and vehicle commands, including internally generated mission commands.
Checking only external commands would leave the incident path unprotected.
Rejected requests are discarded rather than held until liftoff.
A pending forward request accepted while disarmed is also cleared before it can initiate a transition after arming on the ground.

Admission and completion checks serve different purposes and both must remain:

- Rejecting initiation prevents the transition controller from applying transition pitch/thrust on the ground.
- Rejecting completion while armed and landed prevents an immediate handover to fixed-wing controllers, even if normal completion criteria are satisfied.

Disarmed mode changes remain available for ground checks.
Return to multicopter on the ground remains allowed for recovery; the existing `can_transition_on_ground()` helper retains that back-transition use and must not be reused for forward-transition completion.
Normal airborne transition criteria remain in effect.

## Upstream Context

These references explain the history and why the local guards must not be removed merely to match an older upstream implementation:

- [PX4 issue #24082](https://github.com/PX4/PX4-Autopilot/issues/24082) reports the same class of VTOL mission-resume failure after an aborted transition and landing.
- [PX4 PR #21710](https://github.com/PX4/PX4-Autopilot/pull/21710) documents mission-resume improvements, including restoration of camera/gimbal state, and the change to reset on disarm only after reaching the last mission item.
  Survey continuity explains retaining progress, but does not establish that a later flight is in a suitable state to execute it automatically.
- [PX4 PR #27045](https://github.com/PX4/PX4-Autopilot/pull/27045) discusses resetting mission progress on restart.
  The local implementation and tests, rather than that proposal's existence, provide the protection described here.
- [PX4 PR #4479](https://github.com/PX4/PX4-Autopilot/pull/4479) introduced the direct ground-transition behaviour.
  Disarmed checks remain supported without treating armed-and-landed as permission to enter fixed-wing control.

## Regression Coverage And Merge Requirements

Keep these tests with any refactor or upstream replacement:

- `src/modules/navigator/test/MissionStartTest.cpp`: booted checkpoints, disarm/rearm, same-flight pause, explicit airborne resume, required climb before transition, ground rejection, unchanged-index validation, invalid indices, item-zero starts, and loop reset ordering.
- `src/modules/vtol_att_control/VtolGroundTransitionTest.cpp`: internal and external commands, RC requests, discarded ground requests, requests latched before arming, disarmed checks, airborne transitions, ground return to multicopter, and landed completion rejection.
- Existing `functional-test_mission_base` and `functional-test_RTL` suites cover shared mission infrastructure and RTL behaviour.

After configuring the Linux `px4_sitl_test` build, run:

```sh
cmake --build build/px4_sitl_test --target functional-MissionStart functional-VtolGroundTransition functional-test_mission_base functional-test_RTL modules__commander
ctest --test-dir build/px4_sitl_test --output-on-failure -R '^functional-(MissionStart|VtolGroundTransition|test_mission_base|test_RTL)$'
```

On 3 October 2026, the four test executables passed all 76 individual tests, including 18 new regression tests, and Commander compiled successfully using `px4io/px4-dev:v1.17.0`.
A broader `functional-TailsitterHandoff` build was blocked by an existing `-Werror=float-equal` failure in its test source.
These results do not include a complete simulated flight or hardware flight validation.

When resolving merge conflicts, verify the first Mission entry after both reboot and rearm, explicit resume followed by another disarm, both VTOL command paths, and the independent initiation/completion guards.
Keep the operating instructions in the MC, FW, and VTOL Mission guides aligned with the implementation.
