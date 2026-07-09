#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_dir = get_package_share_directory("px4_mid360")
    fast_lio_params = os.path.join(
        project_dir, "config", "fast_lio", "mid360_warehouse.yaml"
    )

    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value="./bags/warehouse_mapping/",
        description="Path to the ros2 bag directory",
    )

    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="./maps/warehouse_map.pcd",
        description="PCD file written by map_saver",
    )

    save_directory_arg = DeclareLaunchArgument(
        "save_directory",
        default_value="./maps",
        description="Fallback directory when map_path is not set",
    )
    fast_lio_package_arg = DeclareLaunchArgument(
        "fast_lio_package",
        default_value="fast_lio2",
        description="FAST-LIO ROS 2 package name",
    )
    fast_lio_executable_arg = DeclareLaunchArgument(
        "fast_lio_executable",
        default_value="fastlio_mapping",
        description="FAST-LIO mapping executable name",
    )

    fast_lio2 = Node(
        package=LaunchConfiguration("fast_lio_package"),
        executable=LaunchConfiguration("fast_lio_executable"),
        name="fast_lio2",
        parameters=[fast_lio_params],
        output="screen",
    )

    odometry_relay = Node(
        package="fast_lio_bridge",
        executable="odometry_relay",
        name="odometry_relay",
        output="screen",
    )

    map_saver = Node(
        package="map_manager",
        executable="map_saver",
        name="map_saver",
        output="screen",
        parameters=[{
            "map_file": LaunchConfiguration("map_path"),
            "save_directory": LaunchConfiguration("save_directory"),
            "cloud_frame_mode": "world",
        }],
    )

    bag_play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            "--clock",
            "--rate",
            "1.0",
            LaunchConfiguration("bag_path"),
        ],
        output="screen",
    )

    return LaunchDescription([
        bag_path_arg,
        map_path_arg,
        save_directory_arg,
        fast_lio_package_arg,
        fast_lio_executable_arg,
        fast_lio2,
        odometry_relay,
        map_saver,
        TimerAction(period=3.0, actions=[bag_play]),
    ])
