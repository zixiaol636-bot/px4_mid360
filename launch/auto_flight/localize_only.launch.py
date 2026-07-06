#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
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

    odometry_relay = Node(
        package="fast_lio_bridge",
        executable="odometry_relay",
        name="odometry_relay",
        output="screen",
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

    return LaunchDescription([
        map_path_arg,
        TimerAction(period=0.0, actions=[fast_lio2, odometry_relay]),
        TimerAction(period=3.0, actions=[map_loader, relocalizer]),
    ])
