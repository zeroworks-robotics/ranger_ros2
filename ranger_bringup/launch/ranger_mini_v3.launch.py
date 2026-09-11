import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
                                             description='Use simulation clock if true')

    port_name_arg = DeclareLaunchArgument('port_name', default_value='can0',
                                         description='CAN bus name, e.g. can0')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
                                           description='Odometry frame id')
    base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link',
                                                description='Base link frame id')
    odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
                                           description='Odometry topic name')

    simulated_robot_arg = DeclareLaunchArgument('simulated_robot', default_value='false',
                                                   description='Whether running with simulator')
    publish_odom_tf_arg = DeclareLaunchArgument('publish_odom_tf', default_value='false',
                                                 description='Simulation control loop update rate')
    update_rate_arg = DeclareLaunchArgument('update_rate', default_value='50',
                                                 description='Simulation control loop update rate')
    robot_model_arg = DeclareLaunchArgument('robot_model', default_value='ranger_mini_v3',
                                           description='robot motion model')
    steer_mode_arg = DeclareLaunchArgument('steer_mode', default_value='twist',
                                           description="'twist' (yaw-rate) or 'direct' (RC-like steering angle)")
    battery_soc_unit_arg = DeclareLaunchArgument('battery_soc_unit', default_value='percent',
                                           description="Unit of /battery_state.percentage: 'percent' (0~100, as the chassis reports) or 'ratio' (0~1, as sensor_msgs documents)")
    command_mode_retry_period_arg = DeclareLaunchArgument('command_mode_retry_period', default_value='1.0',
                                           description='Seconds between re-sending the CAN command mode request while the chassis reports standby; <= 0 disables')
    ranger_base_node = launch_ros.actions.Node(
        package='ranger_base',
        executable='ranger_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'port_name': launch.substitutions.LaunchConfiguration('port_name'),                
                'odom_frame': launch.substitutions.LaunchConfiguration('odom_frame'),
                'base_frame': launch.substitutions.LaunchConfiguration('base_frame'),
                'odom_topic_name': launch.substitutions.LaunchConfiguration('odom_topic_name'),
                'simulated_robot': launch.substitutions.LaunchConfiguration('simulated_robot'),
                'publish_odom_tf': launch.substitutions.LaunchConfiguration('publish_odom_tf'),
                'update_rate': launch.substitutions.LaunchConfiguration('update_rate'),
                'robot_model': launch.substitutions.LaunchConfiguration('robot_model'),
                'steer_mode': launch.substitutions.LaunchConfiguration('steer_mode'),
                'battery_soc_unit': launch.substitutions.LaunchConfiguration('battery_soc_unit'),
                'command_mode_retry_period': launch.substitutions.LaunchConfiguration('command_mode_retry_period'),

        }])

    return LaunchDescription([
        use_sim_time_arg,
        port_name_arg,        
        odom_frame_arg,
        base_link_frame_arg,
        odom_topic_arg,
        simulated_robot_arg,
        publish_odom_tf_arg,
        update_rate_arg,
        robot_model_arg,
        steer_mode_arg,
        battery_soc_unit_arg,
        command_mode_retry_period_arg,
        ranger_base_node
    ])
