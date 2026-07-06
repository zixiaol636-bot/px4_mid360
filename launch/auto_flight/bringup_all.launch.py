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
    nav2_params = os.path.join(project_dir, "config", "nav2", "nav2_params.yaml")
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="./maps/warehouse_map.pcd",
        description="Path to pre-built PCD map",
    )

    map_grid_path_arg = DeclareLaunchArgument(
        "map_grid_path",
        default_value="./maps/map.yaml",
        description="Path to Nav2 occupancy map YAML",
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

    mavros2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory("mavros2"),
                "launch",
                "px4.launch.py",
            ])
        ])
    )

    fast_lio2 = Node(
        package="fast_lio2",
        executable="fastlio_mapping",
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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            ])
        ]),
        launch_arguments={
            "params_file": nav2_params,
            "map": LaunchConfiguration("map_grid_path"),
        }.items(),
    )

    offboard_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory("px4_offboard_bridge"),
                "launch",
                "offboard_bridge.launch.py",
            ])
        ])
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
        map_grid_path_arg,
        TimerAction(period=0.0, actions=[mid360_driver, mavros2]),
        TimerAction(period=2.0, actions=[fast_lio2, fast_lio_bridge]),
        TimerAction(period=5.0, actions=[map_loader, relocalizer]),
        TimerAction(period=8.0, actions=[nav2]),
        TimerAction(
            period=10.0,
            actions=[offboard_bridge, safety_monitors],
        ),
    ])
