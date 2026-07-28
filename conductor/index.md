# Conductor - ranger_ros2

Navigation hub for project context.

## Quick Links

- [Product Definition](./product.md)
- [Product Guidelines](./product-guidelines.md)
- [Tech Stack](./tech-stack.md)
- [Workflow](./workflow.md)
- [Tracks](./tracks.md)

## Code Style Guides

- [C++](./code_styleguides/cpp.md)
- [Python (launch files)](./code_styleguides/python.md)

## Active Tracks

<!-- Auto-populated by /conductor:new-track -->

| Status | Track | Summary |
| --- | --- | --- |
| Pending | [motion-mode-verify_20260728](./tracks/motion-mode-verify_20260728/index.md) | Verify the twist-derived motion-mode selection on the real chassis and tune its thresholds so the mode never chatters. Requires hardware. |
| Pending | [light-control_20260728](./tracks/light-control_20260728/index.md) | Add `/light_control` (`std_msgs/Bool`) to switch the chassis lights on and off. Phase 1 is a hardware go/no-go gate. |

## At a Glance

- **What:** ROS 2 driver for AgileX / Weston Robot Ranger mobile robot bases.
- **Stack:** C++14, ROS 2 Humble, `ament_cmake`, `ugv_sdk` over CAN.
- **Packages:** `ranger_base` (driver), `ranger_msgs` (interfaces), `ranger_bringup` (launch).
- **Deployment:** built and run on the robot PC against a CAN-to-USB adapter.
- **Commits:** the developer commits their own work — Conductor does not commit.
- **Verification:** manual checkpoint after each phase (build → launch → on-robot behavior).

## Getting Started

Run `/conductor:new-track` to create your first feature track.
