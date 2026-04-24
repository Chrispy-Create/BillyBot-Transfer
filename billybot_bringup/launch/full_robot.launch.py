"""
BillyBot — full robot launch file.

Starts everything needed for normal operation:
  1. YDLidar X4 driver
  2. Open-loop odometry (cmd_vel → odom TF)
  3. SLAM Toolbox (online async mapping)
  4. Safety node (obstacle stop via ultrasonics)
  5. Ultrasonic bridge (Arduino Mega HC-SR04 × 12)
  6. EKF + navsat_transform (GPS + odom fusion)
  7. Static TF: base_link → laser_frame

The micro-ROS agent is NOT launched here — it runs as a systemd service
(micro_ros_agent.service) and must be started before this launch file.

Run:
    ros2 launch billybot_bringup full_robot.launch.py

Optional overrides:
    ros2 launch billybot_bringup full_robot.launch.py arduino_port:=/dev/ttyACM1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config_dir = os.path.join(
        get_package_share_directory('billybot_bringup'), 'config'
    )

    # ── Launch arguments ──────────────────────────────────────────────────────
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyACM1',
        description='Serial port for the Arduino Mega ultrasonic sensor board'
    )
    arduino_port = LaunchConfiguration('arduino_port')

    # ── Static TF: base_link → laser_frame ───────────────────────────────────
    static_tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame']
    )

    # ── Static TF: base_link → gps_link ──────────────────────────────────────
    static_tf_base_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_gps',
        # Adjust x/y/z to match where the GPS antenna sits on the robot
        arguments=['0.0', '0.0', '0.3', '0', '0', '0', 'base_link', 'gps_link']
    )

    # ── Open-loop odometry ────────────────────────────────────────────────────
    open_loop_odom = Node(
        package='billybot_bringup',
        executable='open_loop_odom',
        name='open_loop_odom',
        output='screen'
    )

    # ── YDLidar ──────────────────────────────────────────────────────────────
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch', 'ydlidar_launch.py'
            )
        ),
        launch_arguments={
            'params_file': os.path.join(config_dir, 'X4.yaml')
        }.items()
    )

    # ── SLAM Toolbox ──────────────────────────────────────────────────────────
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.join(config_dir, 'slam_config.yaml'),
            'use_sim_time': 'false'
        }.items()
    )

    # ── Ultrasonic bridge (Arduino) ───────────────────────────────────────────
    ultrasonic_bridge = Node(
        package='billybot_bringup',
        executable='ultrasonic_bridge',
        name='ultrasonic_bridge',
        output='screen',
        parameters=[{'serial_port': arduino_port, 'baud_rate': 115200}]
    )

    # ── Safety node ───────────────────────────────────────────────────────────
    safety_node = Node(
        package='billybot_bringup',
        executable='safety_node',
        name='billybot_safety',
        output='screen'
    )

    # ── EKF (local odometry filter) ───────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'ekf_config.yaml')],
        remappings=[('odometry/filtered', 'odometry/filtered')]
    )

    # ── navsat_transform (GPS lat/lon → odom frame) ───────────────────────────
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[os.path.join(config_dir, 'ekf_config.yaml')],
        remappings=[
            ('gps/fix',             'gps/fix'),
            ('odometry/filtered',   'odometry/filtered'),
            ('odometry/gps',        'odometry/gps'),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(arduino_port_arg)
    ld.add_action(static_tf_base_laser)
    ld.add_action(static_tf_base_gps)
    ld.add_action(open_loop_odom)
    ld.add_action(ydlidar_launch)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(ultrasonic_bridge)
    ld.add_action(safety_node)
    ld.add_action(ekf_node)
    ld.add_action(navsat_node)
    return ld
