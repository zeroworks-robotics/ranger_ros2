# C++ Style Guide - ranger_ros2

Conventions derived from the existing source in `ranger_base/`. When in doubt, match the
surrounding code.

## Language Standard

**C++14**, set in `ranger_base/CMakeLists.txt`. Do not use C++17-only features
(structured bindings, `if constexpr`, `std::optional`) unless the standard is bumped
first as a deliberate change.

## Formatting

- **Indentation:** 2 spaces, no tabs.
- **Line length:** 80 columns. The existing code wraps at 80; keep it there.
- **Braces:** K&R — opening brace on the same line, including for functions.
  ```cpp
  void RangerROSMessenger::TwistCmdCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
    if (parking_mode_) {
      return;
    }
  }
  ```
- **Always brace conditionals**, even single-statement ones.
- **Pointer/reference binding:** attach to the type — `double& radius`, not `double &radius`.

## Naming

| Kind | Convention | Example |
| --- | --- | --- |
| Class / struct | `PascalCase` | `RangerROSMessenger`, `RobotParams` |
| Method | `PascalCase` | `TwistCmdCallback`, `CalculateSteeringAngle` |
| Member variable | `snake_case_` with **trailing underscore** | `motion_mode_`, `spin_leave_vx_`, `robot_params_` |
| Local variable | `snake_case`, no underscore | `steer_cmd`, `has_rot` |
| Parameter | `snake_case` | `angle`, `dt` |
| Constant / enum value | `kPascalCase` (SDK style) | `kRangerMiniV2`, `kDualAckerman` |
| Message constant | `SCREAMING_SNAKE_CASE` | `MOTION_MODE_DUAL_ACKERMAN` |
| ROS parameter | `snake_case` matching the member without the underscore | member `cmd_deadzone_` ↔ param `"cmd_deadzone"` |

The trailing-underscore rule for members is load-bearing here — it is the only thing
distinguishing `motion_mode_` (driver state) from a local. Do not omit it.

## Class Layout

Follow the order already in `ranger_messenger.hpp`:

1. Nested types (`struct RobotParams`, `enum class RangerSubType`)
2. `public:` — constructor, `Run()`
3. `private:` — methods, then members
4. Members grouped and commented by role: parameters, motion-mode state, publishers,
   subscribers, services, odometry state

Initialize members **in-class** at the declaration, and make the default match the ROS
parameter default:

```cpp
double cmd_deadzone_ = 1e-2;          // m/s, rad/s; |axis| at or below this counts as zero
double mode_switch_min_dwell_ = 0.6;  // s; hold a mode at least this long after switching
```

## Comments

The comment style here is unusually rationale-heavy, and that is deliberate. Preserve it.

- **Explain why, not what.** A comment restating the code is noise; a comment explaining
  that a switch costs the chassis ~0.6 s of steering reconfiguration is the reason the
  code exists.
- **Document units** on every physical quantity: `// m/s`, `// rad`, `// s`.
- **Block comments** above a non-obvious decision, using the section-header form already
  in use for major stages:
  ```cpp
  // ── Motion-mode selection from the commanded twist ───────────────────────
  ```
- **Sentinel values must be explained** at their definition — e.g. `0xFF` meaning
  "nothing commanded yet".
- Use `//`, not `/* */`, for anything other than the file header.
- **Do not leave commented-out code.** Delete it; git remembers.

## ROS 2 Conventions

- **Parameters:** declare in `LoadParameters()` via
  `node_->declare_parameter<T>("name", default)`, assigning to the matching member.
  Every parameter gets a safe default equal to the in-class initializer.
- **Callbacks:** named `<Thing>Callback`, bound with `std::bind(..., this,
  std::placeholders::_1)`.
- **Subscriptions/publishers:** created in `SetupSubscription()`, stored in
  `..._sub_` / `..._pub_` members.
- **Logging:** `RCLCPP_INFO/WARN/ERROR(node_->get_logger(), ...)`. Do not log at INFO on
  a per-message path — a 50 Hz callback logging every cycle floods the console. Log on
  state *changes* only.
- **Message constants:** refer to modes via the generated constants
  (`MotionState::MOTION_MODE_SPINNING`), never a bare integer literal.

## Numeric and Safety Rules

These are specific to this driver and matter more than general style.

- **Never compare floats with `==`/`!=` for a decision.** Use the deadzone:
  ```cpp
  const bool has_x = std::abs(msg->linear.x) > cmd_deadzone_;   // good
  if (msg->linear.x != 0.0) { ... }                             // bad
  ```
- **Guard every division and `atan`** against a zero/zero input. `atan(y/x)` with both
  zero is NaN, and a NaN reaching `SetMotionCommand()` is a hardware command.
- **Clamp before commanding.** Steering angles clamp to
  `robot_params_.max_steer_angle_*`, speeds to `max_linear_speed` /
  `max_angular_speed`. Never send an unclamped value to `ugv_sdk`.
- **Gate `SetMotionMode()` on an actual change.** It is expensive; calling it every
  callback stalls the chassis.
- **Variant differences are explicit.** Guard on `robot_type_`; never assume a variant.

## Headers

- Include guards: `#ifndef RANGER_MESSENGER_HPP` / `#define` / `#endif  // RANGER_MESSENGER_HPP`.
- Include order, matching existing files: std/C++ → ROS → third-party → project messages
  → project headers, with a blank line and a short comment between groups.
- Keep the driver class in `westonrobot` namespace.

---

_Generated by Conductor. Review and edit as needed._
