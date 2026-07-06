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
        project_dir, "config", "fast_lio", "mid360_warehouse.yaml"
    )

    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="./maps/warehouse_online_map.pcd",
        description="PCD file written by map_saver during live mapping",
    )

    save_directory_arg = DeclareLaunchArgument(
        "save_directory",
        default_value="./maps",
        description="Fallback save directory when map_path is not set",
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
        parameters=[fast_lio_params],
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

    map_saver = Node(
        package="map_manager",
        executable="map_saver",
        name="map_saver",
        parameters=[{
            "map_file": LaunchConfiguration("map_path"),
            "save_directory": LaunchConfiguration("save_directory"),
        }],
        output="screen",
    )

    return LaunchDescription([
        map_path_arg,
        save_directory_arg,
        TimerAction(period=0.0, actions=[mid360_driver, mavros2]),
        TimerAction(period=2.0, actions=[fast_lio2, fast_lio_bridge]),
        TimerAction(period=5.0, actions=[map_saver]),
    ])
