#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("px4_offboard_bridge")
    default_params = os.path.join(pkg_dir, "config", "offboard_params.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Parameter YAML file for PX4 XRCE-DDS bridge nodes",
    )
    enable_ego_follower_arg = DeclareLaunchArgument(
        "enable_ego_planner_path_follower",
        default_value="false",
        description="Optional fallback follower for planners that publish nav_msgs/Path on /planning/local_path",
    )
    enable_safety_filter_arg = DeclareLaunchArgument(
        "enable_local_3d_safety_filter",
        default_value="true",
        description="Insert final 3D safety filter before PX4 setpoints",
    )
    params_file = LaunchConfiguration("params_file")

    ego_planner_path_follower_node = Node(
        package="px4_offboard_bridge",
        executable="ego_planner_path_follower",
        name="ego_planner_path_follower",
        output="screen",
        parameters=[params_file],
        respawn=True,
        condition=IfCondition(LaunchConfiguration("enable_ego_planner_path_follower")),
    )

    local_3d_safety_filter_node = Node(
        package="px4_offboard_bridge",
        executable="local_3d_safety_filter",
        name="local_3d_safety_filter",
        output="screen",
        parameters=[params_file],
        respawn=True,
        condition=IfCondition(LaunchConfiguration("enable_local_3d_safety_filter")),
    )

    px4_visual_odometry_bridge_node = Node(
        package="px4_offboard_bridge",
        executable="px4_visual_odometry_bridge",
        name="px4_visual_odometry_bridge",
        output="screen",
        parameters=[params_file],
        respawn=True,
    )

    px4_cmdvel_bridge_node = Node(
        package="px4_offboard_bridge",
        executable="px4_cmdvel_bridge",
        name="px4_cmdvel_bridge",
        output="screen",
        parameters=[params_file],
        respawn=True,
    )

    px4_status_bridge_node = Node(
        package="px4_offboard_bridge",
        executable="px4_status_bridge",
        name="px4_status_bridge",
        output="screen",
        parameters=[params_file],
        respawn=True,
    )

    return LaunchDescription([
        params_arg,
        enable_ego_follower_arg,
        enable_safety_filter_arg,
        ego_planner_path_follower_node,
        local_3d_safety_filter_node,
        px4_visual_odometry_bridge_node,
        px4_cmdvel_bridge_node,
        px4_status_bridge_node,
    ])
