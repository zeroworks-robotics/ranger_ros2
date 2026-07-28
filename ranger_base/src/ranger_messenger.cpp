/**
* @file ranger_messenger.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include "ranger_base/ranger_messenger.hpp"

#include "ranger_base/kinematics_model.hpp"

using namespace rclcpp;
using namespace ranger_msgs::msg;

namespace westonrobot {
// namespace {
// double DegreeToRadian(double x) { return x * M_PI / 180.0; }
// }  // namespace

namespace {
// TEMPORARY (track motion-mode-verify_20260728, Task 1.3) — remove in Task 4.3
// together with the [mode-diag] logging in TwistCmdCallback.
const char* MotionModeName(uint8_t mode) {
  switch (mode) {
    case MotionState::MOTION_MODE_DUAL_ACKERMAN: return "DUAL_ACKERMAN";
    case MotionState::MOTION_MODE_PARALLEL:      return "PARALLEL";
    case MotionState::MOTION_MODE_SPINNING:      return "SPINNING";
    case MotionState::MOTION_MODE_PARKING:       return "PARKING";
    case MotionState::MOTION_MODE_SIDE_SLIP:     return "SIDE_SLIP";
    case 0xFF:                                   return "NONE";
    default:                                     return "UNKNOWN";
  }
}
}  // namespace

///////////////////////////////////////////////////////////////////////////////////
RangerROSMessenger::RangerROSMessenger(rclcpp::Node::SharedPtr& node){

  node_ = node;
  LoadParameters();

  // connect to robot and setup ROS subscription
  if (robot_type_ == RangerSubType::kRangerMiniV1) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV1);
  } else if (robot_type_ == RangerSubType::kRangerMiniV2) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV2);
  } else if (robot_type_ == RangerSubType::kRangerMiniV3) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV3);
  } else {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRanger);
  }

  if (port_name_.find("can") != std::string::npos) {
    if (!robot_->Connect(port_name_)) {
      RCLCPP_ERROR(node_->get_logger(),"Failed to connect to the CAN port");
      return;
    }
    robot_->EnableCommandedMode();
  } else {
    RCLCPP_ERROR(node_->get_logger(),"Invalid port name: %s", port_name_.c_str());
    return;
  }

  SetupSubscription();
}

void RangerROSMessenger::Run() {
  rclcpp::Rate rate(update_rate_);
  while (rclcpp::ok()) {
    PublishStateToROS();
    rclcpp::spin_some(node_);
    rate.sleep();
  }
}

void RangerROSMessenger::LoadParameters() {
  //load parameter from launch files
  port_name_ = node_->declare_parameter<std::string>("port_name","can0");
  robot_model_ = node_->declare_parameter<std::string>("robot_model","ranger");
  odom_frame_ =  node_->declare_parameter<std::string>("odom_frame","odom");
  base_frame_ = node_->declare_parameter<std::string>("base_frame", "base_link");
  update_rate_ = node_->declare_parameter<int>("update_rate", 50);
  odom_topic_name_ = node_->declare_parameter<std::string>("odom_topic_name", "odom");
  publish_odom_tf_ = node_->declare_parameter<bool>("publish_odom_tf",false);
  // "twist"  : angular.z is a yaw rate, steering is derived from the kinematics
  // "direct" : angular.z is a steering angle (rad), RC-like, speed-independent
  std::string steer_mode = node_->declare_parameter<std::string>("steer_mode", "twist");
  direct_steer_ = (steer_mode == "direct");

  // Twist-driven motion-mode selection (see TwistCmdCallback): deadzone below
  // which a /cmd_vel axis counts as zero, plus anti-chatter thresholds.
  cmd_deadzone_ = node_->declare_parameter<double>("cmd_deadzone", 1e-2);
  mode_switch_min_dwell_ =
      node_->declare_parameter<double>("mode_switch_min_dwell", 0.6);
  spin_leave_vx_ = node_->declare_parameter<double>("spin_leave_vx", 3e-2);
  last_mode_switch_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(),
      "Successfully loaded the following parameters: \n port_name: %s\n "
      "robot_model: %s\n odom_frame: %s\n base_frame: %s\n "
      "update_rate: %d\n odom_topic_name: %s\n "
      "publish_odom_tf: %d\n",
      port_name_.c_str(), robot_model_.c_str(), odom_frame_.c_str(),
      base_frame_.c_str(), update_rate_, odom_topic_name_.c_str(),
      publish_odom_tf_);

  // load robot parameters
  if (robot_model_ == "ranger_mini_v1") {
    robot_type_ = RangerSubType::kRangerMiniV1;

    robot_params_.track = RangerMiniV1Params::track;
    robot_params_.wheelbase = RangerMiniV1Params::wheelbase;
    robot_params_.max_linear_speed = RangerMiniV1Params::max_linear_speed;
    robot_params_.max_angular_speed = RangerMiniV1Params::max_angular_speed;
    robot_params_.max_speed_cmd = RangerMiniV1Params::max_speed_cmd;
    robot_params_.max_steer_angle_central =
        RangerMiniV1Params::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel =
        RangerMiniV1Params::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerMiniV1Params::max_round_angle;
    robot_params_.min_turn_radius = RangerMiniV1Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV1Params::max_steer_angle_ackermann;
  } else {
    if (robot_model_ == "ranger_mini_v2") {
      robot_type_ = RangerSubType::kRangerMiniV2;

      robot_params_.track = RangerMiniV2Params::track;
      robot_params_.wheelbase = RangerMiniV2Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV2Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV2Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV2Params::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerMiniV2Params::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerMiniV2Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV2Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV2Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV2Params::max_steer_angle_ackermann;
    }
    if (robot_model_ == "ranger_mini_v3") {
      robot_type_ = RangerSubType::kRangerMiniV3;

      robot_params_.track = RangerMiniV3Params::track;
      robot_params_.wheelbase = RangerMiniV3Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV3Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV3Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV3Params::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerMiniV3Params::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerMiniV3Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV3Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV3Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV3Params::max_steer_angle_ackermann;
    }
     else {
      robot_type_ = RangerSubType::kRanger;

      robot_params_.track = RangerParams::track;
      robot_params_.wheelbase = RangerParams::wheelbase;
      robot_params_.max_linear_speed = RangerParams::max_linear_speed;
      robot_params_.max_angular_speed = RangerParams::max_angular_speed;
      robot_params_.max_speed_cmd = RangerParams::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerParams::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerParams::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerParams::max_round_angle;
      robot_params_.min_turn_radius = RangerParams::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerParams::max_steer_angle_ackermann;
    }
  }
    parking_mode_ = false;
    commanded_motion_mode_ = 0xFF;  // force the first cmd to issue SetMotionMode

}

void RangerROSMessenger::SetupSubscription() {
  // publisher
  system_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::SystemState>("/system_state", 10);
  motion_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::MotionState>("/motion_state", 10);
  actuator_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::ActuatorStateArray>("/actuator_state", 10);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 10);
  battery_state_pub_ =
      node_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);

  // subscriber
  motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 5, std::bind(&RangerROSMessenger::TwistCmdCallback, this, std::placeholders::_1)
      );

  // control-mode command (raw CAN 0x421 value) from cmd_vel_manager
  control_mode_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/cmd_vel_manager/control_mode", 5,
      std::bind(&RangerROSMessenger::ControlModeCallback, this, std::placeholders::_1)
      );

  // service to enter/leave the chassis parking mode
  set_parking_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "/set_parking_mode",
      std::bind(&RangerROSMessenger::SetParkingModeCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

void RangerROSMessenger::ControlModeCallback(
    std_msgs::msg::UInt8::SharedPtr msg) {
  // Forward the raw control-mode value straight to the chassis (CAN 0x421).
  RCLCPP_INFO(node_->get_logger(), "Setting chassis control mode: %u",
              static_cast<unsigned int>(msg->data));
  robot_->SetControlMode(msg->data);
}

void RangerROSMessenger::SetParkingModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  // Parking mode is only supported on Ranger Mini V2/V3 (matches the guard in
  // TwistCmdCallback and the feedback tracking in PublishStateToROS).
  if (robot_type_ != RangerSubType::kRangerMiniV2 &&
      robot_type_ != RangerSubType::kRangerMiniV3) {
    response->success = false;
    response->message = "Parking mode is only supported on Ranger Mini V2/V3";
    return;
  }

  if (request->data) {
    // enter parking; parking_mode_ is set from chassis feedback once the
    // switch is confirmed, which then makes TwistCmdCallback drop /cmd_vel.
    robot_->SetMotionMode(RangerInterface::MotionMode::kPark);
    response->message = "Entering parking mode";
  } else {
    // leave parking; fall back to dual-ackermann. The next /cmd_vel will
    // re-select the appropriate motion mode.
    robot_->SetMotionMode(RangerInterface::MotionMode::kDualAckerman);
    response->message = "Leaving parking mode";
  }
  response->success = true;
}

void RangerROSMessenger::PublishStateToROS() {
  current_time_ = node_->get_clock()->now();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = robot_->GetRobotState();
  auto actuator_state = robot_->GetActuatorState();

  // update odometry
  {
    double dt = (current_time_ - last_time_).seconds();
    UpdateOdometry(state.motion_state.linear_velocity,
                   state.motion_state.angular_velocity,
                   state.motion_state.steering_angle, dt);
    last_time_ = current_time_;
  }

  // publish system state
  {
    ranger_msgs::msg::SystemState system_msg;
    system_msg.header.stamp = current_time_;
    system_msg.vehicle_state = state.system_state.vehicle_state;
    system_msg.control_mode = state.system_state.control_mode;
    system_msg.error_code = state.system_state.error_code;
    system_msg.battery_voltage = state.system_state.battery_voltage;
    system_msg.motion_mode = state.motion_mode_state.motion_mode;

    system_state_pub_->publish(system_msg);
  }

  // publish motion mode
  {
    motion_mode_ = state.motion_mode_state.motion_mode;

    // Track the chassis hardware parking mode reported over CAN (0x291). The
    // chassis enters/leaves parking externally (e.g. via the RC controller);
    // ROS only observes it here. TwistCmdCallback uses parking_mode_ to drop
    // motion commands while parked (applied to Ranger Mini V2/V3, see guard).
    parking_mode_ = (state.motion_mode_state.motion_mode ==
                     RangerInterface::MotionMode::kPark);

    ranger_msgs::msg::MotionState motion_msg;
    motion_msg.header.stamp = current_time_;
    motion_msg.motion_mode = state.motion_mode_state.motion_mode;

    motion_state_pub_->publish(motion_msg);
  }

  // publish actuator state
  {
    // RCLCPP_DEBUG(node_->get_logger(),"feedback", "Angle_5:%f Angle_6:%f Angle_7:%f Angle_8:%f",
    //                 actuator_state.motor_angles.angle_5,
    //                 actuator_state.motor_angles.angle_6,
    //                 actuator_state.motor_angles.angle_7,
    //                 actuator_state.motor_angles.angle_8);
    // RCLCPP_DEBUG(node_->get_logger(),"feedback", "speed_1:%f speed_2:%f speed_3:%f speed_4:%f",
    //                 actuator_state.motor_speeds.speed_1,
    //                 actuator_state.motor_speeds.speed_2,
    //                 actuator_state.motor_speeds.speed_3,
    //                 actuator_state.motor_speeds.speed_4);

    ranger_msgs::msg::ActuatorStateArray actuator_msg;
    actuator_msg.header.stamp = current_time_;

    // Ranger has 4 wheels, each with a drive motor and a steering motor (8 in
    // total). speed_1~4 map to drive motors (actuator id 0~3) and angle_5~8 map
    // to steering motors (actuator id 4~7).
    const float motor_speeds[8] = {actuator_state.motor_speeds.speed_1,
                                   actuator_state.motor_speeds.speed_2,
                                   actuator_state.motor_speeds.speed_3,
                                   actuator_state.motor_speeds.speed_4,
                                   0, 0, 0, 0};
    const float motor_angles[8] = {0, 0, 0, 0,
                                   actuator_state.motor_angles.angle_5,
                                   actuator_state.motor_angles.angle_6,
                                   actuator_state.motor_angles.angle_7,
                                   actuator_state.motor_angles.angle_8};

    for (int i = 0; i < 8; i++) {
      ranger_msgs::msg::DriverState driver_state_msg;
      driver_state_msg.driver_voltage =
          actuator_state.actuator_ls_state[i].driver_voltage;
      driver_state_msg.driver_temperature =
          actuator_state.actuator_ls_state[i].driver_temp;
      driver_state_msg.motor_temperature =
          actuator_state.actuator_ls_state[i].motor_temp;
      driver_state_msg.driver_state =
          actuator_state.actuator_ls_state[i].driver_state;

      ranger_msgs::msg::MotorState motor_state_msg;
      motor_state_msg.current = actuator_state.actuator_hs_state[i].current;
      motor_state_msg.pulse_count = actuator_state.actuator_hs_state[i].pulse_count;
      motor_state_msg.rpm = actuator_state.actuator_hs_state[i].rpm;
      motor_state_msg.motor_angles = motor_angles[i];
      motor_state_msg.motor_speeds = motor_speeds[i];

      ranger_msgs::msg::ActuatorState actuator_state_msg;
      actuator_state_msg.id = i;
      actuator_state_msg.driver = driver_state_msg;
      actuator_state_msg.motor = motor_state_msg;

      actuator_msg.states.push_back(actuator_state_msg);
    }

    actuator_state_pub_->publish(actuator_msg);
  }

  // publish BMS state
  {
    auto common_sensor_state = robot_->GetCommonSensorState();

    sensor_msgs::msg::BatteryState batt_msg;
    batt_msg.header.stamp = current_time_;
    batt_msg.voltage = common_sensor_state.bms_basic_state.voltage;
    batt_msg.temperature = common_sensor_state.bms_basic_state.temperature;
    batt_msg.current = common_sensor_state.bms_basic_state.current;
    batt_msg.percentage = common_sensor_state.bms_basic_state.battery_soc;
    batt_msg.charge = std::numeric_limits<float>::quiet_NaN();
    batt_msg.capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batt_msg.power_supply_health =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batt_msg.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batt_msg.present = std::numeric_limits<uint8_t>::quiet_NaN();

    battery_state_pub_->publish(batt_msg);
  }
}

void RangerROSMessenger::UpdateOdometry(double linear, double angular,
                                        double angle, double dt) {
  // update odometry calculations
  if (motion_mode_ == MotionState::MOTION_MODE_DUAL_ACKERMAN) {
    DualAckermanModel::state_type x = {position_x_, position_y_, theta_};
    DualAckermanModel::control_type u;
    u.v = linear;
    u.phi = ConvertInnerAngleToCentral(angle);

    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<DualAckermanModel::state_type>(),
        DualAckermanModel(robot_params_.wheelbase, u), x, 0.0, dt, (dt / 10.0));
    //std::cout<<" steer: "<<angle<<" central: "<<u.phi<<std::endl;
    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  } else if (motion_mode_ == MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
    ParallelModel::state_type x = {position_x_, position_y_, theta_};
    ParallelModel::control_type u;
    u.v = linear;
    if (motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
      u.phi = M_PI / 2.0;
    } else {
      u.phi = angle;
    }
    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<ParallelModel::state_type>(),
        ParallelModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  } else if (motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
    SpinningModel::state_type x = {position_x_, position_y_, theta_};
    SpinningModel::control_type u;
    u.w = angular;

    boost::numeric::odeint::integrate_const(
        boost::numeric::odeint::runge_kutta4<SpinningModel::state_type>(),
        SpinningModel(u), x, 0.0, dt, (dt / 10.0));

    position_x_ = x[0];
    position_y_ = x[1];
    theta_ = x[2];
  }

  // update odometry topics
  geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(theta_);

  // publish odometry and tf messages
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = position_x_;
  odom_msg.pose.pose.position.y = position_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  if (motion_mode_ == MotionState::MOTION_MODE_DUAL_ACKERMAN) {
    odom_msg.twist.twist.linear.x = linear;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z =
        2 * linear * std::sin(ConvertInnerAngleToCentral(angle)) /
        robot_params_.wheelbase;
  } else if (motion_mode_ == MotionState::MOTION_MODE_PARALLEL ||
             motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
    double phi = angle;

    if (motion_mode_ == MotionState::MOTION_MODE_SIDE_SLIP) {
      phi = M_PI / 2.0;
    }
    odom_msg.twist.twist.linear.x = linear * std::cos(phi);
    odom_msg.twist.twist.linear.y = linear * std::sin(phi);

    odom_msg.twist.twist.angular.z = 0;
  } else if (motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
    odom_msg.twist.twist.linear.x = 0;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = angular;
  }

  odom_pub_->publish(odom_msg);

  // // publish tf transformation
  if (publish_odom_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void RangerROSMessenger::TwistCmdCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
  double steer_cmd = 0.0;
  double radius = 0.0;

  // Parking mode (set via the /set_parking_mode service) halts commands on
  // Mini V2/V3, regardless of the mode selection below.
  if (parking_mode_ &&
      (robot_type_ == RangerSubType::kRangerMiniV2 ||
       robot_type_ == RangerSubType::kRangerMiniV3)) {
    return;
  }

  // Steering angle for the DUAL_ACKERMAN case.
  if (direct_steer_) {
    steer_cmd = msg->angular.z;
  } else {
    steer_cmd = CalculateSteeringAngle(*msg, radius);
  }

  // ── Motion-mode selection from the commanded twist ───────────────────────
  // The mode is derived here from /cmd_vel itself, so it works the same for
  // every command source (joystick via cmd_vel_manager, teleop, nav stack):
  //
  //   lateral component        -> parallel (side-slip on Mini V1 with no vx)
  //   rotation, no forward vel -> spinning (turn in place)
  //   rotation + forward vel   -> dual-ackerman (follow an arc)
  //   forward velocity only    -> dual-ackerman
  //   everything at rest       -> hold the current mode
  //
  // Axes at or below cmd_deadzone_ count as zero, so command noise around zero
  // can't trigger a mode switch.
  const bool has_x = std::abs(msg->linear.x) > cmd_deadzone_;
  const bool has_y = std::abs(msg->linear.y) > cmd_deadzone_;
  const bool has_rot = std::abs(msg->angular.z) > cmd_deadzone_;

  if (direct_steer_) {
    // RC-like: angular.z is a steering-angle command, not a yaw rate, so there
    // is no spin case; stay in dual-ackerman and let the angle steer the arc.
    motion_mode_ = MotionState::MOTION_MODE_DUAL_ACKERMAN;
  } else if (has_y) {
    // lateral component requested: V1 with no forward speed uses the dedicated
    // side-slip mode; every other case uses parallel steering (pure lateral
    // motion on non-V1 robots is handled inside the PARALLEL case below).
    if (!has_x && robot_type_ == RangerSubType::kRangerMiniV1) {
      motion_mode_ = MotionState::MOTION_MODE_SIDE_SLIP;
    } else {
      motion_mode_ = MotionState::MOTION_MODE_PARALLEL;
    }
  } else if (has_rot) {
    // Spin in place only when no forward/backward motion is commanded. With a
    // linear velocity present (e.g. teleop sending linear.x and angular.z
    // together), stay in dual-ackerman and follow an arc so the linear
    // component is not dropped.
    //
    // Hysteresis on the ackermann<->spinning boundary: entering spin needs |vx|
    // within the deadzone, but leaving it needs |vx| to rise past a higher
    // threshold (spin_leave_vx_), so a velocity that lingers near zero (a
    // deceleration tail, approach jitter) can't flip the mode back and forth.
    if (commanded_motion_mode_ == MotionState::MOTION_MODE_SPINNING) {
      motion_mode_ = (std::abs(msg->linear.x) > spin_leave_vx_)
                         ? MotionState::MOTION_MODE_DUAL_ACKERMAN
                         : MotionState::MOTION_MODE_SPINNING;
    } else {
      motion_mode_ = has_x ? MotionState::MOTION_MODE_DUAL_ACKERMAN
                           : MotionState::MOTION_MODE_SPINNING;
    }
  } else if (has_x) {
    motion_mode_ = MotionState::MOTION_MODE_DUAL_ACKERMAN;
  } else {
    // Nothing commanded: hold the mode the chassis is already in rather than
    // falling back to dual-ackerman, so releasing the sticks between two
    // movements doesn't cost a mode switch (~0.6 s of steering reconfiguration).
    motion_mode_ = (commanded_motion_mode_ == 0xFF)
                       ? MotionState::MOTION_MODE_DUAL_ACKERMAN
                       : commanded_motion_mode_;
  }

  // Only switch modes when the target actually changes. Re-sending the same
  // mode every callback is wasted CAN traffic; more importantly a real switch
  // makes the chassis reconfigure its steering for ~0.6 s (during which it
  // ignores speed commands), so we must not keep re-issuing it.
  //
  // Minimum dwell: once we switch, hold the new mode for at least
  // mode_switch_min_dwell_ seconds. A brief excursion across the decision
  // boundary within that window keeps commanding the current mode instead of
  // thrashing the chassis. 0xFF ("nothing commanded yet") always switches.
  // ── TEMPORARY diagnostics (track motion-mode-verify_20260728, Task 1.3) ───
  // Instruments the driver's *intent* for the on-hardware boundary sweeps.
  // /motion_state carries chassis feedback, not what this callback decided, so
  // without this there is no way to tell a driver-side flip from chassis lag.
  //
  // Deliberately transition-only: this callback runs at the /cmd_vel rate, and
  // logging every message would flood the console and perturb the very timing
  // the sweeps are measuring. Remove in Task 4.3.
  //
  // Dwell-suppressed switches are logged too — Phase 3 Task 3.4 needs them to
  // distinguish a properly damped boundary from one that only looks stable
  // because the dwell timer is masking chatter underneath.
  static uint8_t diag_last_blocked_target = 0xFE;  // 0xFE = "nothing blocked"
  if (motion_mode_ != commanded_motion_mode_) {
    if (commanded_motion_mode_ == 0xFF ||
        (node_->now() - last_mode_switch_time_).seconds() >=
            mode_switch_min_dwell_) {
      RCLCPP_INFO(node_->get_logger(),
                  "[mode-diag] %.3f SWITCH %s -> %s | vx=%+.4f vy=%+.4f "
                  "wz=%+.4f",
                  node_->now().seconds(),
                  MotionModeName(commanded_motion_mode_),
                  MotionModeName(motion_mode_), msg->linear.x, msg->linear.y,
                  msg->angular.z);
      diag_last_blocked_target = 0xFE;

      robot_->SetMotionMode(motion_mode_);
      commanded_motion_mode_ = motion_mode_;
      last_mode_switch_time_ = node_->now();
    } else {
      // Log only when the blocked target itself changes, not on every cycle of
      // the dwell window, which would otherwise emit tens of lines per block.
      if (motion_mode_ != diag_last_blocked_target) {
        diag_last_blocked_target = motion_mode_;
        RCLCPP_INFO(node_->get_logger(),
                    "[mode-diag] %.3f DWELL-BLOCKED %s -> %s (%.3f s of %.3f s)"
                    " | vx=%+.4f vy=%+.4f wz=%+.4f",
                    node_->now().seconds(),
                    MotionModeName(commanded_motion_mode_),
                    MotionModeName(motion_mode_),
                    (node_->now() - last_mode_switch_time_).seconds(),
                    mode_switch_min_dwell_, msg->linear.x, msg->linear.y,
                    msg->angular.z);
      }
      // still within the dwell window: keep the current mode this cycle
      motion_mode_ = commanded_motion_mode_;
    }
  }

  // send motion command to robot
  switch (motion_mode_) {
    case MotionState::MOTION_MODE_DUAL_ACKERMAN: {
      if (steer_cmd > robot_params_.max_steer_angle_ackermann) {
        steer_cmd = robot_params_.max_steer_angle_ackermann;
      }
      if (steer_cmd < -robot_params_.max_steer_angle_ackermann) {
        steer_cmd = -robot_params_.max_steer_angle_ackermann;
      }
      robot_->SetMotionCommand(msg->linear.x, steer_cmd);
      break;
    }
    case MotionState::MOTION_MODE_PARALLEL: {
      // atan(y/x) is nan when both are zero (possible now that PARALLEL can be
      // forced via /cmd_vel_manager with no velocity); treat that as 0 steering.
      steer_cmd = (msg->linear.x == 0.0 && msg->linear.y == 0.0)
                      ? 0.0
                      : atan(msg->linear.y / msg->linear.x);

      static double last_nonzero_x = 1.0; 
      
      if (msg->linear.x != 0.0) {
          last_nonzero_x = msg->linear.x; 
      }

      if (std::signbit(msg->linear.x))
      {
        steer_cmd = -steer_cmd;
      }
      
      if (steer_cmd > robot_params_.max_steer_angle_parallel) {
        steer_cmd = robot_params_.max_steer_angle_parallel;
      }
      if (steer_cmd < -robot_params_.max_steer_angle_parallel) {
        steer_cmd = -robot_params_.max_steer_angle_parallel;
      }
      double vel = 1.0;
      
      if (msg->linear.x == 0.0 && msg->linear.y != 0.0) {
          // std::cout << "MOTION_MODE_SIDE_SLIP" << std::endl;
          
          if (std::signbit(last_nonzero_x)) {
              steer_cmd = -std::abs(steer_cmd); 
          } else {
              steer_cmd = std::abs(steer_cmd);
          }
          vel = msg->linear.y >= 0 ? 1.0 : -1.0;
      } else {
          vel = msg->linear.x >= 0 ? 1.0 : -1.0;
      }
      robot_->SetMotionCommand(vel * sqrt(msg->linear.x * msg->linear.x +
                                          msg->linear.y * msg->linear.y),
                               steer_cmd);
      break;
    }
    case MotionState::MOTION_MODE_SPINNING: {
      double a_v = msg->angular.z;
      if (a_v > robot_params_.max_angular_speed) {
        a_v = robot_params_.max_angular_speed;
      }
      if (a_v < -robot_params_.max_angular_speed) {
        a_v = -robot_params_.max_angular_speed;
      }
      robot_->SetMotionCommand(0.0, 0.0, a_v);
      break;
    }
    case MotionState::MOTION_MODE_SIDE_SLIP: {
      double l_v = msg->linear.y;
      if (l_v > robot_params_.max_linear_speed) {
        l_v = robot_params_.max_linear_speed;
      }
      if (l_v < -robot_params_.max_linear_speed) {
        l_v = -robot_params_.max_linear_speed;
      }
      robot_->SetMotionCommand(0.0, 0.0, l_v);
      break;
    }
  }
}


geometry_msgs::msg::Quaternion RangerROSMessenger::createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

double RangerROSMessenger::CalculateSteeringAngle(geometry_msgs::msg::Twist msg,
                                                  double& radius) {
  double linear = std::abs(msg.linear.x);
  double angular = std::abs(msg.angular.z);

  if (angular < 1e-6) {
    radius = std::numeric_limits<double>::infinity(); 
    return 0.0; 
  }
  // Circular motion
  radius = linear / angular;
  int k = (msg.angular.z * msg.linear.x) >= 0 ? 1 : -1;

  const double l = robot_params_.wheelbase;
  double phi_i = atan((l / 2) / radius);

  // clamp to the model's maximum (inner) steering angle
  phi_i = std::min(phi_i, robot_params_.max_steer_angle_ackermann);

  return k * phi_i;
}

double RangerROSMessenger::ConvertInnerAngleToCentral(double angle) {
  double phi = 0;
  double phi_i = std::abs(angle);

  phi = std::atan(robot_params_.wheelbase * std::sin(phi_i) /
                  (robot_params_.wheelbase * std::cos(phi_i) +
                   robot_params_.track * std::sin(phi_i)));

  phi *= angle >= 0 ? 1.0 : -1.0;
  return phi;
}

double RangerROSMessenger::ConvertCentralAngleToInner(double angle) {
  double phi = std::abs(angle);
  double phi_i = 0;

  phi_i = std::atan(robot_params_.wheelbase * std::sin(phi) /
                    (robot_params_.wheelbase * std::cos(phi) -
                     robot_params_.track * std::sin(phi)));
  phi_i *= angle >= 0 ? 1.0 : -1.0;
  return phi_i;
}
}  // namespace westonrobot
