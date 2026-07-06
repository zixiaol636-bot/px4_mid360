#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_frame_arg = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="Global frame used for map accumulation",
    )

    odom_frame_arg = DeclareLaunchArgument(
        "odom_frame",
        default_value="odom",
        description="Odometry frame id",
    )

    save_directory_arg = DeclareLaunchArgument(
        "save_directory",
        default_value="/tmp",
        description="Directory used when map_file is not provided",
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="",
        description="Optional fixed PCD output file path",
    )

    voxel_leaf_size_arg = DeclareLaunchArgument(
        "voxel_leaf_size",
        default_value="0.1",
        description="Voxel size used while building the map",
    )

    map_saver_node = Node(
        package="map_manager",
        executable="map_saver",
        name="map_saver",
        output="screen",
        parameters=[{
            "map_frame": LaunchConfiguration("map_frame"),
            "odom_frame": LaunchConfiguration("odom_frame"),
            "save_directory": LaunchConfiguration("save_directory"),
            "map_file": LaunchConfiguration("map_file"),
            "voxel_leaf_size": LaunchConfiguration("voxel_leaf_size"),
        }],
    )

    return LaunchDescription([
        map_frame_arg,
        odom_frame_arg,
        save_directory_arg,
        map_file_arg,
        voxel_leaf_size_arg,
        map_saver_node,
    ])
