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
        description="Path to YAML parameter file",
    )
    enable_offboard_controller_arg = DeclareLaunchArgument(
        "enable_offboard_controller",
        default_value="false",
        description="Whether to launch the demo offboard takeoff controller",
    )
    enable_z_axis_monitor_arg = DeclareLaunchArgument(
        "enable_z_axis_monitor",
        default_value="false",
        description="Whether to launch the legacy Z-axis only obstacle monitor",
    )

    params_file = LaunchConfiguration("params_file")
    enable_offboard_controller = LaunchConfiguration("enable_offboard_controller")
    enable_z_axis_monitor = LaunchConfiguration("enable_z_axis_monitor")

    vision_pose_bridge_node = Node(
        package="px4_offboard_bridge",
        executable="vision_pose_bridge",
        name="vision_pose_bridge",
        output="screen",
        parameters=[params_file],
        respawn=True,
    )

    offboard_controller_node = Node(
        package="px4_offboard_bridge",
        executable="offboard_controller",
        name="offboard_controller",
        output="screen",
        parameters=[params_file],
        respawn=False,
        condition=IfCondition(enable_offboard_controller),
    )

    cmdvel_to_setpoint_node = Node(
        package="px4_offboard_bridge",
        executable="cmdvel_to_setpoint",
        name="cmdvel_to_setpoint",
        output="screen",
        parameters=[params_file],
        respawn=True,
    )

    local_3d_avoidance_node = Node(
        package="px4_offboard_bridge",
        executable="local_3d_avoidance",
        name="local_3d_avoidance",
        output="screen",
        parameters=[params_file],
        respawn=True,
    )

    z_axis_monitor_node = Node(
        package="px4_offboard_bridge",
        executable="z_axis_monitor",
        name="z_axis_monitor",
        output="screen",
        parameters=[params_file],
        respawn=True,
        condition=IfCondition(enable_z_axis_monitor),
    )

    return LaunchDescription([
        params_arg,
        enable_offboard_controller_arg,
        enable_z_axis_monitor_arg,
        vision_pose_bridge_node,
        offboard_controller_node,
        local_3d_avoidance_node,
        cmdvel_to_setpoint_node,
        z_axis_monitor_node,
    ])
