#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    project_dir = get_package_share_directory("px4_mid360")
    default_waypoint_file = os.path.join(
        project_dir, "config", "waypoints", "warehouse_example.yaml"
    )

    waypoint_file_arg = DeclareLaunchArgument(
        "waypoint_file",
        default_value=default_waypoint_file,
        description="Path to waypoint YAML file",
    )

    takeoff_altitude_arg = DeclareLaunchArgument(
        "takeoff_altitude",
        default_value="5.0",
        description="Climb height above mission start pose before waypoint navigation",
    )

    waypoint_tolerance_arg = DeclareLaunchArgument(
        "waypoint_arrival_tolerance",
        default_value="1.0",
        description="Default waypoint acceptance radius when not set in YAML",
    )

    navigate_timeout_arg = DeclareLaunchArgument(
        "max_navigate_timeout",
        default_value="120.0",
        description="Maximum allowed seconds per waypoint leg before RTL",
    )

    mission_executor = Node(
        package="waypoint_planner",
        executable="mission_executor",
        name="mission_executor",
        parameters=[{
            "waypoints_file": LaunchConfiguration("waypoint_file"),
            "takeoff_altitude": ParameterValue(
                LaunchConfiguration("takeoff_altitude"), value_type=float
            ),
            "waypoint_arrival_tolerance": ParameterValue(
                LaunchConfiguration("waypoint_arrival_tolerance"), value_type=float
            ),
            "max_navigate_timeout": ParameterValue(
                LaunchConfiguration("max_navigate_timeout"), value_type=float
            ),
        }],
        output="screen",
    )

    mission_start = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/mission/start",
            "std_srvs/srv/Trigger",
            "{}",
        ],
        output="screen",
    )

    return LaunchDescription([
        waypoint_file_arg,
        takeoff_altitude_arg,
        waypoint_tolerance_arg,
        navigate_timeout_arg,
        LogInfo(msg="=== Starting autonomous mission executor ==="),
        mission_executor,
        TimerAction(period=3.0, actions=[mission_start]),
    ])
