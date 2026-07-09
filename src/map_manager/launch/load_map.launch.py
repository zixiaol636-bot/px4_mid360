#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="/tmp/map.pcd",
        description="PCD map file to load",
    )

    map_frame_arg = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="Frame id for the published map cloud",
    )

    load_on_start_arg = DeclareLaunchArgument(
        "load_on_start",
        default_value="true",
        description="Whether to load and publish the map on startup",
    )

    map_loader_node = Node(
        package="map_manager",
        executable="map_loader",
        name="map_loader",
        output="screen",
        parameters=[{
            "map_file": LaunchConfiguration("map_file"),
            "map_frame": LaunchConfiguration("map_frame"),
            "load_on_start": ParameterValue(
                LaunchConfiguration("load_on_start"), value_type=bool),
        }],
    )

    return LaunchDescription([
        map_file_arg,
        map_frame_arg,
        load_on_start_arg,
        map_loader_node,
    ])
