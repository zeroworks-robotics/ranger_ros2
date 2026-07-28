# Specification: Verify cmd_vel-Driven Motion-Mode Selection

**Track ID:** motion-mode-verify_20260728
**Type:** Feature
**Created:** 2026-07-28
**Status:** Draft

## Summary

Motion-mode selection was moved out of `cmd_vel_manager` and into the `ranger_base`
driver, where it is now derived from the commanded twist on `/cmd_vel`. This track
verifies that logic on the real chassis and tunes its thresholds so the mode never
chatters at any decision boundary.

## Context

A Ranger chassis is four-wheel independent-steer and exposes several distinct motion
modes (`DUAL_ACKERMAN`, `PARALLEL`, `SPINNING`, `SIDE_SLIP`, `PARKING`). Switching
between them is physically expensive: the chassis spends roughly 0.6 s reconfiguring its
steering and ignores speed commands while it does. A selection rule that flips modes
near a threshold therefore does not merely look wrong on a topic — it stalls the robot
and is felt by the operator.

Selection previously lived in `cmd_vel_manager`, which computed the mode from raw
joystick axes and published it on `/cmd_vel_manager/motion_state` for the driver to
apply. That worked only for joystick input; any other `/cmd_vel` publisher got whatever
mode was last commanded. The logic now lives in `ranger_base::TwistCmdCallback` and
reads the twist directly, so every command source is treated identically.

The implementation is written but **has never been run against hardware**. Its damping
mechanisms — a command deadzone, hysteresis on the ackermann↔spinning boundary, a
minimum dwell between switches, and holding the current mode when nothing is commanded —
all carry default thresholds that were reasoned about rather than measured. Verifying
and tuning them on the chassis is the purpose of this track.

## User Story

As an **internal robotics engineer**, I want the driver's motion-mode selection verified
and tuned on the real chassis, so that I can trust the robot to hold a stable mode while
driving instead of stuttering whenever a velocity command lingers near a threshold.

## Decision Boundaries Under Test

The selection logic in `TwistCmdCallback` has these boundaries. Each must be swept and
confirmed chatter-free.

| # | Boundary | Governing value |
| --- | --- | --- |
| 1 | Deadzone on `linear.x` — moving vs. at rest | `cmd_deadzone` (0.01 m/s) |
| 2 | Deadzone on `linear.y` — lateral onset, selects `PARALLEL` | `cmd_deadzone` (0.01 m/s) |
| 3 | Deadzone on `angular.z` — rotation onset | `cmd_deadzone` (0.01 rad/s) |
| 4 | `DUAL_ACKERMAN` ↔ `SPINNING` | enter at `cmd_deadzone`, leave at `spin_leave_vx` (0.03 m/s) |
| 5 | ~~`SIDE_SLIP` vs. `PARALLEL`~~ — **N/A** | Mini V1 only; see below |
| 6 | Idle hold — all axes in deadzone | last commanded mode retained |
| 7 | Any switch, globally | `mode_switch_min_dwell` (0.6 s) |

**Chassis under test: Ranger Mini V3.** Boundary 5 is not reachable — `SIDE_SLIP` is
selected only when `robot_type_ == kRangerMiniV1`, so on a V3 a pure-lateral twist always
yields `PARALLEL`. Six boundaries are therefore in scope, not seven.

**Parking mode is live on V3.** The guard at the top of `TwistCmdCallback` drops every
command while `parking_mode_` is set, on Mini V2 and V3 only. `parking_mode_` is driven
by chassis feedback, so anything that parks the chassis — including the RC controller —
silently stops all mode selection with no diagnostic output. Confirm the chassis is not
parked before concluding that a sweep produced no result.

## Acceptance Criteria

- [ ] Sweeping `/cmd_vel` slowly across **each** of the seven boundaries above produces
      no oscillation on `/motion_state` — the mode settles and stays settled.
- [ ] Every motion mode reachable from a twist on a Mini V3 (`DUAL_ACKERMAN`,
      `PARALLEL`, `SPINNING`) is confirmed reachable on the chassis, so each boundary can
      actually be exercised from both sides. `SIDE_SLIP` is out of reach on this variant.
- [ ] A deceleration to a stop from each mode does not trigger a spurious mode change on
      the way down.
- [ ] Final values for `cmd_deadzone`, `spin_leave_vx`, and `mode_switch_min_dwell` are
      determined on hardware and recorded, with the in-class initializer in
      `ranger_messenger.hpp` and the `declare_parameter` default in `LoadParameters()`
      kept in agreement.
- [ ] `ranger_base` builds clean and the node runs for a full driving session without a
      mode-selection-related fault.

## Dependencies

**Depends on existing, uncommitted code.**

1. `ranger_ros2` (this repo, branch `features/add_manager`) — modified
   `ranger_base/src/ranger_messenger.cpp` and
   `ranger_base/include/ranger_base/ranger_messenger.hpp`. Contains the twist-derived
   selection, the `cmd_deadzone` parameter, and the removal of the
   `/cmd_vel_manager/motion_state` subscriber.
2. **`cmd_vel_manager` — separate git repository**, at
   `~/work/docker/cona_ws/src/cmd_vel_manager`, branch `features/webot_added`. Its
   motion-mode publisher, `select_motion_mode`, the button-8 `robot_mode` toggle, and
   the `ranger_msgs` dependency were removed. These edits are **also uncommitted** and
   are in scope for verification, though they live outside this repository.

Both working trees must be present and built for end-to-end verification. Neither has
been committed, so the track's first job is to confirm the tree is in the expected state.

**Hardware:** a real Ranger chassis with a working CAN-to-USB adapter is required. Per
the answers gathered, `ranger_can_sim` is not used for this track — mode-switch timing
and the reconfiguration stall are hardware behaviors.

## Out of Scope

- **Restoring a manual mode override.** No topic, service, or button will be re-added to
  force a motion mode. The commanded twist is the sole source of truth. This is revisited
  only if verification proves the twist genuinely insufficient — and that would be a new
  track, not a change of scope here.
- **Velocity ramping or blending across a mode switch.** Actively smoothing the chassis's
  ~0.6 s reconfiguration is a separate concern; this track verifies and tunes *when* the
  mode changes, not what the velocity does while it changes.
- Parking mode (`/set_parking_mode`) behavior and its chassis-feedback tracking.
- `UpdateOdometry` and the kinematic models in `kinematics_model.hpp`, even though they
  branch on `motion_mode_`.
- The `direct_steer_` (`steer_mode: direct`) path, which bypasses mode selection and
  always commands `DUAL_ACKERMAN`.

## Technical Notes

- **Observability first.** The chassis reports its own mode on `/motion_state` at
  `update_rate` (50 Hz). That topic is the primary instrument: chatter is visible as
  oscillation there. Note that `PublishStateToROS()` overwrites `motion_mode_` from
  chassis feedback every cycle, so `/motion_state` reflects the *chassis*, not the
  driver's intent — distinguishing a driver-side flip from a chassis-side lag may require
  temporary logging of `commanded_motion_mode_`.
- **Do not log per-message at INFO.** A 50 Hz callback logging every cycle floods the
  console and will itself perturb timing. Log on transitions only.
- The three damping mechanisms interact. `mode_switch_min_dwell` can mask chatter that
  the deadzone should have prevented, so test with dwell temporarily lowered to isolate
  whether the deadzone and hysteresis are correctly sized on their own.
- Tuning must keep the header initializer and the `declare_parameter` default in sync;
  a disagreement between them is a silent "behaves differently under launch" bug.
- Behavior deliberately changed from the `cmd_vel_manager` implementation in two places,
  and both should be confirmed as acceptable on hardware:
  - pure lateral input now selects `PARALLEL`/`SIDE_SLIP` automatically, where previously
    it required the button-8 toggle;
  - an all-zero twist now holds the current mode, where previously it reverted to the
    toggle's `robot_mode`.

---

_Generated by Conductor. Review and edit as needed._
