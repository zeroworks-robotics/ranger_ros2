# Specification: Light On/Off Topic

**Track ID:** light-control_20260728
**Type:** Feature
**Created:** 2026-07-28
**Status:** Draft

## Summary

Add a ROS topic that turns the Ranger chassis lights on and off. The driver subscribes to
`/light_control` (`std_msgs/Bool`) and forwards the request to the chassis over CAN via
`ugv_sdk`.

## Context

The Ranger chassis supports light control over CAN, and `ugv_sdk` already exposes it —
but `ranger_base` does not use it. There is currently no light-related code in the driver
at all: no subscriber, no publisher, and the `light_state` field that the chassis reports
back in `RangerCoreState` is read into the state struct and then ignored.

The SDK surface is:

```cpp
void SetLightCommand(AgxLightMode f_mode, uint8_t f_value,
                     AgxLightMode r_mode, uint8_t r_value);
```

with front and rear lights controlled independently, and modes:

| Mode | Value | Meaning |
| --- | --- | --- |
| `CONST_OFF` | `0x00` | Off |
| `CONST_ON` | `0x01` | On |
| `BREATH` | `0x02` | Pulsing |
| `CUSTOM` | `0x03` | Brightness given by the accompanying `custom_value` byte |

This track deliberately uses only `CONST_ON` and `CONST_OFF`. A simple boolean on/off is
what is needed; exposing the full mode and brightness surface is not.

## User Story

As an **internal robotics engineer**, I want to switch the robot's lights on and off from
a ROS topic, so that lighting can be controlled from software — by an operator UI, a
teleop node, or an autonomous behavior — without touching the chassis directly.

## Interface

**Subscribes**

| Topic | Type | Behavior |
| --- | --- | --- |
| `/light_control` | `std_msgs/Bool` | `true` → both front and rear lights `CONST_ON`; `false` → both `CONST_OFF`. |

No new message package is introduced — `std_msgs` is already a dependency of
`ranger_base`.

## Acceptance Criteria

- [ ] Publishing `true` to `/light_control` turns the chassis lights on; publishing
      `false` turns them off. Confirmed visually on the robot.
- [ ] The command is idempotent — repeatedly publishing the same value does not cause
      flicker or produce redundant CAN traffic.
- [ ] The driver does not log at INFO on every received message; light state changes are
      logged, repeats are not.
- [ ] `ranger_base` builds clean, and the light subscriber does not interfere with the
      motion command path — a driving session behaves identically with and without light
      commands being sent.
- [ ] If the chassis variant under test has no controllable lights, that is determined
      and recorded explicitly rather than left as an unexplained non-result.

## Dependencies

- **`ugv_sdk`** — provides `RangerRobot::SetLightCommand()` and the `AgxLightMode` enum
  in `ugv_sdk/details/interface/agilex_types.h`. Already a dependency of `ranger_base`;
  no new packages required.
- **`std_msgs`** — already a dependency.
- **Hardware** — a Ranger chassis with controllable lights. See the risk below.
- No dependency on `motion-mode-verify_20260728`; the two tracks touch disjoint code and
  can be done in either order.

## Out of Scope

- `BREATH` and `CUSTOM` (brightness) modes.
- Independent front/rear control — this track sets both lights together.
- Publishing chassis light state back to ROS. The chassis does report `light_state` in
  `RangerCoreState`, and echoing it on a `/light_state` topic is an obvious follow-up,
  but it is not required for on/off control and is not included here.
- A `ranger_msgs/LightCommand` message type.
- Any light behavior tied to robot state (e.g. lights following motion mode, brake
  lights, turn indicators).

## Technical Notes

- **Light presence is not guaranteed.** `RangerRobot::DisableLightControl()` in
  `ugv_sdk/mobile_robot/ranger_robot.hpp` is a no-op stub carrying the comment
  *"do nothing if no light on robot"*, which implies not every Ranger variant has
  controllable lights. Confirm on the actual chassis early — before building out the
  code path — so a hardware limitation is not mistaken for a software bug.
- **Command-control enable.** `LightCommandMessage` carries an `enable_cmd_ctrl` flag
  alongside the front/rear operations. The chassis may ignore light commands unless
  command control is enabled; `EnableCommandedMode()` is already called at connect time
  in the driver's constructor, but whether that is sufficient for the light path
  specifically needs checking against the SDK's `SendLightCommand` implementation.
- **Follow the existing callback pattern.** The subscriber belongs in
  `SetupSubscription()` alongside `motion_cmd_sub_` and `control_mode_sub_`, with a
  `LightControlCallback` method and a `light_control_sub_` member, matching the naming
  already used in `ranger_messenger.hpp`.
- **Track last-commanded state** in a member so repeat messages can be suppressed, the
  same way `commanded_motion_mode_` gates `SetMotionMode()`. This satisfies the
  idempotence criterion and keeps CAN traffic down.
- Light control is independent of parking mode and of the motion-mode logic; it must not
  be gated on either.

---

_Generated by Conductor. Review and edit as needed._
