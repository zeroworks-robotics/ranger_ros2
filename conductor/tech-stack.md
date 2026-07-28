# Tech Stack - ranger_ros2

## Languages

| Language | Where | Notes |
| --- | --- | --- |
| **C++14** | `ranger_base` driver node | Set in `ranger_base/CMakeLists.txt` (`CMAKE_CXX_STANDARD 14` when not already defined). |
| **Python 3** | `ranger_base/launch/`, `ranger_bringup/launch/` | ROS 2 Python launch files (`*.launch.py`). |
| **Bash** | `ranger_bringup/scripts/` | CAN-to-USB adapter setup (`setup_can2usb.bash`, `bringup_can2usb.bash`). |

## Framework

**ROS 2 Humble** (`rclcpp`, `ament_cmake`).

- Build system: `colcon` + `ament_cmake`.
- Interface generation: `rosidl_default_generators` in `ranger_msgs`.
- The repository also carries a `jazzy` branch; `humble` is the default branch and the
  one this workspace tracks.

## Core Dependencies

| Dependency | Purpose |
| --- | --- |
| **`ugv_sdk`** | AgileX chassis SDK. Owns the CAN protocol — `RangerRobot`, `SetMotionCommand`, `SetMotionMode`, `SetControlMode` (CAN `0x421`), and the chassis state structs. Sibling package in the workspace. |
| **`Boost.odeint`** | Runge-Kutta 4 integration of the per-mode kinematic models in `ranger_base/include/ranger_base/kinematics_model.hpp` (dual-ackerman, parallel, spinning). |
| **`Eigen3`** | Linear algebra (`eigen3/Eigen/Core`). |
| **`tf2` / `tf2_ros` / `tf2_geometry_msgs`** | Odometry TF broadcast (`odom` → `base_link`) and quaternion conversion. |
| **`ranger_msgs`** | In-repo message package for Ranger-specific state topics. |
| **`geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`** | Standard ROS interfaces: `Twist` in, `Odometry` / `BatteryState` out, `UInt8` control mode, `SetBool` parking service. |
| **`libasio-dev`, `libboost-all-dev`** | System dependencies (see README). |

## Hardware Interface

The driver talks to the chassis over a **CAN-to-USB adapter** (`can0` by default, set by
the `port_name` parameter). The `gs_usb` kernel module must be loaded, and the interface
brought up via the scripts in `ranger_bringup/scripts/` after each power cycle.

`ranger_can_sim` (sibling package in the workspace) can stand in for real hardware.

## Database

**None.** The driver is stateless — all persistent state lives on the chassis itself.
In-process state is limited to odometry integration and motion-mode bookkeeping.

## Infrastructure

**Built and deployed directly on the robot PC.** The packages are built with `colcon`
into a workspace on the robot's onboard computer and run there against the CAN adapter.

A Docker container is used for some development builds, but the robot PC is the
deployment target and the environment that matters for verification.

## Package Layout

```
ranger_ros2/
├── ranger_base/
│   ├── include/ranger_base/
│   │   ├── ranger_messenger.hpp    # driver class: subs, pubs, mode selection
│   │   ├── kinematics_model.hpp    # dual-ackerman / parallel / spinning models
│   │   └── ranger_params.hpp       # per-variant physical constants
│   ├── src/
│   │   ├── ranger_base_node.cpp    # main()
│   │   └── ranger_messenger.cpp    # driver implementation
│   └── launch/                     # per-variant launch files
├── ranger_msgs/msg/                # MotionState, SystemState, ActuatorState, ...
└── ranger_bringup/
    ├── launch/
    └── scripts/                    # CAN-to-USB setup
```

## ROS Interface

**Subscribes**

| Topic | Type | Purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command. Also the sole input for motion-mode selection. |
| `/cmd_vel_manager/control_mode` | `std_msgs/UInt8` | Raw chassis control mode (CAN `0x421`); 3 = drive, 1 = standby. |

**Publishes**

| Topic | Type |
| --- | --- |
| `/system_state` | `ranger_msgs/SystemState` |
| `/motion_state` | `ranger_msgs/MotionState` |
| `/actuator_state` | `ranger_msgs/ActuatorStateArray` |
| `odom` (configurable) | `nav_msgs/Odometry` |
| `/battery_state` | `sensor_msgs/BatteryState` |

**Services**

| Service | Type | Purpose |
| --- | --- | --- |
| `/set_parking_mode` | `std_srvs/SetBool` | Enter/leave chassis parking (Ranger Mini V2/V3 only). |

## Key Parameters

| Parameter | Default | Purpose |
| --- | --- | --- |
| `port_name` | `can0` | CAN interface. |
| `robot_model` | `ranger` | Variant: `ranger`, `ranger_mini_v1`, `ranger_mini_v2`, `ranger_mini_v3`. |
| `odom_frame` / `base_frame` | `odom` / `base_link` | TF frames. |
| `odom_topic_name` | `odom` | Odometry topic. |
| `publish_odom_tf` | `false` | Whether to broadcast `odom` → `base_link`. |
| `update_rate` | `50` | Main loop rate (Hz). |
| `steer_mode` | `twist` | `twist` = `angular.z` is a yaw rate; `direct` = `angular.z` is a steering angle (RC-like). |
| `cmd_deadzone` | `0.01` | Below this magnitude a `/cmd_vel` axis counts as zero for mode selection. |
| `mode_switch_min_dwell` | `0.6` | Seconds a mode is held after switching. |
| `spin_leave_vx` | `0.03` | Leave spinning only once \|vx\| exceeds this (hysteresis). |

## Testing

There is currently **no test suite**. `ament_lint_auto` is present in
`ranger_base/CMakeLists.txt` but its block is commented out and should be re-enabled
(see `workflow.md`). Verification today is build + on-robot behavior, optionally against
`ranger_can_sim`.

---

_Generated by Conductor. Review and edit as needed._
