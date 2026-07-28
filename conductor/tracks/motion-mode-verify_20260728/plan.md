# Implementation Plan: Verify cmd_vel-Driven Motion-Mode Selection

**Track ID:** motion-mode-verify_20260728
**Spec:** [spec.md](./spec.md)
**Created:** 2026-07-28
**Status:** [ ] Not Started

## Overview

The selection logic is already written; this track is about proving it on hardware and
settling its thresholds. The plan therefore front-loads **observability** — without a way
to distinguish a driver-side mode flip from chassis feedback lag, every later measurement
is ambiguous.

The approach is: instrument, confirm every mode is reachable, then sweep each decision
boundary in isolation with the dwell timer temporarily disarmed so the deadzone and
hysteresis have to stand on their own, then restore the dwell and lock in final values.

**On testing:** per `workflow.md` (moderate TDD), unit tests are wanted for
self-contained logic like mode selection — but this repository has no test harness at
all, and standing one up is explicitly a different track. The selection logic is also
not currently extractable as a pure function; it reads and writes member state inside
`TwistCmdCallback`. This track therefore verifies on hardware only. Extracting the
selection into a testable pure function is noted as a follow-up, not done here.

**Hardware is required for every phase after 1.** If the chassis is unavailable, stop
and say so rather than marking a phase complete.

## Phase 1: Baseline and Instrumentation

Establish a known-good starting state and the ability to see what the driver is deciding,
as opposed to what the chassis is reporting.

### Tasks

- [x] Task 1.1: Confirm both working trees are in the expected state — `ranger_ros2` on
      `features/add_manager` with modified `ranger_messenger.{hpp,cpp}`, and
      `cmd_vel_manager` on `features/webot_added` with the motion-mode publisher removed.
      Both are uncommitted; record the current diffs before changing anything.
- [x] Task 1.2: Build `ranger_msgs`, `ranger_base`, and `cmd_vel_manager` clean. Resolve
      the known `--symlink-install` conflict on `ranger_msgs` by removing
      `build/ranger_msgs` if it appears.
- [x] Task 1.3: Add **temporary** transition-only logging in `TwistCmdCallback` that
      reports every change of `commanded_motion_mode_` together with the twist that
      caused it (`linear.x`, `linear.y`, `angular.z`) and a timestamp. Transition-only —
      a 50 Hz log floods the console and perturbs the timing being measured. Mark it
      clearly as temporary so Phase 4 can remove it.
- [ ] Task 1.4: **BLOCKED — no hardware.** Bring up the robot against the real CAN
      adapter and confirm the expected topics are present and live: `/motion_state`,
      `/system_state`, `odom`, `/battery_state`, and `/cmd_vel`.
      _Attempted 2026-07-28: no `can*` interface on the host or in the `sm-dev`
      container, and no `gs_usb` / `can_dev` / `can_raw` kernel modules loaded. The
      chassis is not connected. Per `workflow.md`, paused rather than marked complete._

### Verification

- [ ] **BLOCKED on Task 1.4.** Packages build with no new warnings _(done — see below)_;
      the node runs against the chassis, publishes all expected topics, and the temporary
      transition log emits on a mode change and stays silent otherwise _(requires
      hardware)_.

_Build portion confirmed 2026-07-28: `ranger_msgs`, `ranger_base`, and `cmd_vel_manager`
rebuilt from scratch with no new warnings. The only warnings are pre-existing and in
files this track does not touch — `joystick_linux.h` (`-Wreorder`) and
`kinematics_model.hpp` (`-Wunused-parameter`)._

## Phase 2: Mode Reachability

Confirm each mode can actually be commanded before trying to measure its boundaries. A
boundary cannot be swept from both sides if one side is unreachable.

### Tasks

- [ ] Task 2.1: Command each reachable mode from a twist and confirm on `/motion_state`
      that the chassis enters it: `DUAL_ACKERMAN`, `PARALLEL`, `SPINNING`. Record the
      exact twist used for each. (`SIDE_SLIP` is **N/A** — the chassis under test is a
      Mini V3, and side-slip is selected only on a Mini V1.)
- [ ] Task 2.2: Confirm the two deliberate behavior changes from the old
      `cmd_vel_manager` implementation, and get explicit sign-off that each is acceptable:
      pure lateral input now auto-selects `PARALLEL`/`SIDE_SLIP` where it previously
      needed the button-8 toggle, and an all-zero twist now holds the current mode where
      it previously reverted.
- [ ] Task 2.3: Verify end-to-end through `cmd_vel_manager` — a joystick drive session
      selects modes correctly with nothing published on
      `/cmd_vel_manager/motion_state`. Confirm `/cmd_vel_manager/control_mode` still
      gates drive/standby as before.

### Verification

- [ ] Every mode reachable from a twist is confirmed entered on the chassis, the twist
      for each is recorded, and the two behavior changes are signed off.

## Phase 3: Boundary Sweeps

The core of the track. Sweep each of the seven boundaries from the spec and look for
oscillation on `/motion_state` and in the transition log.

### Tasks

- [ ] Task 3.1: Temporarily set `mode_switch_min_dwell` to a near-zero value. The dwell
      timer can mask chatter that the deadzone and hysteresis should have prevented on
      their own; disarming it is what makes the next two tasks meaningful.
- [ ] Task 3.2: Sweep the three deadzone boundaries (1–3 in the spec table) — ramp
      `linear.x`, then `linear.y`, then `angular.z` slowly up through `cmd_deadzone` and
      back down, including holding the command *at* the threshold. Record any oscillation.
- [ ] Task 3.3: Sweep the mode-pair boundaries — the `DUAL_ACKERMAN`↔`SPINNING`
      hysteresis band between `cmd_deadzone` and `spin_leave_vx` (boundary 4), and the
      idle-hold case with all axes inside the deadzone (boundary 6). Include a
      deceleration to a stop from each mode, checking for a spurious switch on the way
      down. (Boundary 5 is **N/A** on a Mini V3.)
- [ ] Task 3.4: Restore `mode_switch_min_dwell` and re-run any sweep that showed
      oscillation, to see whether the dwell alone is carrying it — a boundary that only
      behaves because of the dwell is under-damped and needs its deadzone or hysteresis
      widened instead.

### Verification

- [ ] All six in-scope boundaries swept and results recorded (5 is N/A on a Mini V3). Every boundary either settles
      cleanly with the dwell disarmed, or is identified as needing a threshold change in
      Phase 4.

## Phase 4: Tuning and Finalization

Lock in the values, remove the scaffolding, and confirm the result over a real session.

### Tasks

- [ ] Task 4.1: Set final values for `cmd_deadzone`, `spin_leave_vx`, and
      `mode_switch_min_dwell` based on the Phase 3 measurements. Where a boundary needed
      widening, prefer widening the deadzone or hysteresis over lengthening the dwell.
- [ ] Task 4.2: Keep the in-class initializers in `ranger_messenger.hpp` and the
      `declare_parameter` defaults in `LoadParameters()` in agreement, and check the
      launch files for any hardcoded override that would silently disagree.
- [ ] Task 4.3: Remove the temporary transition logging added in Task 1.3.
- [ ] Task 4.4: Update `conductor/tech-stack.md` with the final parameter values in the
      Key Parameters table, and record the measured values in this track for future
      reference.

### Verification

- [ ] Rebuild clean, then run a full driving session on the chassis covering all reachable
      modes and repeated transitions between them. No mode-selection chatter, no
      mode-selection-related fault, and no temporary logging left in the source.

## Final Verification

- [ ] All acceptance criteria in [spec.md](./spec.md) met
- [ ] Tests passing — n/a, no test harness in this repository (see Overview)
- [ ] `conductor/tech-stack.md` parameter table updated
- [ ] Changes left uncommitted for the developer to review and commit
- [ ] Ready for review

## Follow-ups Identified

- Extract the mode-selection decision into a pure function so it can be unit-tested
  without hardware. Prerequisite for any future test-suite track.
- Velocity ramping across a mode switch, to close key goal #3 ("no perceptible jolt") —
  explicitly out of scope here.

---

_Generated by Conductor. Tasks will be marked [~] in progress and [x] complete._
