import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    config_dir = os.path.join(
        get_package_share_directory('billybot_bringup'),
        'config'
    )
    slam_config_file  = os.path.join(config_dir, 'slam_config.yaml')
    lidar_config_file = os.path.join(config_dir, 'X4.yaml')

    ydlidar_launch_file = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py'
    )

    slam_toolbox_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    # ── Static TF: base_link → laser_frame ──────────────────────────────────
    static_tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.1', '0', '0.2',   # x y z  (metres)
            '0',   '0', '0',     # roll pitch yaw  (radians)
            'base_link',
            'laser_frame'
        ]
    )

    # ── Open-loop odometry (cmd_vel → odom TF) ───────────────────────────────
    # Provides the odom→base_link transform SLAM Toolbox requires.
    # Integrates commanded velocity; will be replaced by real encoder odometry
    # once the hall-sensor differential receiver is wired up.
    open_loop_odom = Node(
        package='billybot_bringup',
        executable='open_loop_odom',
        name='open_loop_odom',
        output='screen'
    )

    # ── YDLidar ─────────────────────────────────────────────────────────────
    start_ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch_file),
        launch_arguments={'params_file': lidar_config_file}.items()
    )

    # ── SLAM Toolbox ─────────────────────────────────────────────────────────
    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={
            'slam_params_file': slam_config_file,
            'use_sim_time': 'false'
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(static_tf_base_laser)
    ld.add_action(open_loop_odom)
    ld.add_action(start_ydlidar)
    ld.add_action(start_slam_toolbox)
    return ld
