#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    project_dir = get_package_share_directory("px4_mid360")
    fast_lio_params = os.path.join(
        project_dir, "config", "fast_lio", "mid360_localization.yaml"
    )
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="./maps/warehouse_map.pcd",
        description="Path to pre-built PCD map",
    )
    enable_ego_planner_arg = DeclareLaunchArgument(
        "enable_ego_planner_ros2",
        default_value="true",
        description="Launch ego-planner-swarm ros2_version for real 3D local planning",
    )
    fast_lio_package_arg = DeclareLaunchArgument(
        "fast_lio_package",
        default_value="fast_lio2",
        description="FAST-LIO ROS 2 package name",
    )
    fast_lio_executable_arg = DeclareLaunchArgument(
        "fast_lio_executable",
        default_value="fastlio_mapping",
        description="FAST-LIO mapping/localization executable name",
    )

    mid360_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory("mid360_driver_wrapper"),
                "launch",
                "mid360_driver.launch.py",
            ])
        ])
    )

    fast_lio2 = Node(
        package=LaunchConfiguration("fast_lio_package"),
        executable=LaunchConfiguration("fast_lio_executable"),
        name="fast_lio2",
        parameters=[
            fast_lio_params,
            {"localization.map_path": LaunchConfiguration("map_path")},
        ],
        output="screen",
    )

    fast_lio_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory("fast_lio_bridge"),
                "launch",
                "odometry_pipeline.launch.py",
            ])
        ])
    )

    map_loader = Node(
        package="map_manager",
        executable="map_loader",
        name="map_loader",
        parameters=[{"map_file": LaunchConfiguration("map_path")}],
        output="screen",
    )

    relocalizer = Node(
        package="relocalizer",
        executable="icp_relocalizer",
        name="icp_relocalizer",
        parameters=[fast_lio_params],
        output="screen",
    )

    offboard_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory("px4_offboard_bridge"),
                "launch",
                "offboard_bridge.launch.py",
            ])
        ]),
        launch_arguments={
            "enable_ego_planner_path_follower": "false",
        }.items(),
    )

    ego_planner_ros2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                project_dir,
                "launch",
                "auto_flight",
                "ego_planner_ros2.launch.py",
            ])
        ]),
        launch_arguments={
            "enable_ego_planner_ros2": LaunchConfiguration("enable_ego_planner_ros2"),
        }.items(),
    )

    safety_monitors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory("safety_monitor"),
                "launch",
                "all_monitors.launch.py",
            ])
        ])
    )

    return LaunchDescription([
        map_path_arg,
        enable_ego_planner_arg,
        fast_lio_package_arg,
        fast_lio_executable_arg,
        TimerAction(period=0.0, actions=[mid360_driver]),
        TimerAction(period=2.0, actions=[fast_lio2, fast_lio_bridge]),
        TimerAction(period=5.0, actions=[map_loader, relocalizer]),
        TimerAction(period=6.0, actions=[ego_planner_ros2]),
        TimerAction(
            period=7.0,
            actions=[offboard_bridge, safety_monitors],
        ),
    ])
