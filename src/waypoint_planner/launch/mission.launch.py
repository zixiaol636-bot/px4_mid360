#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waypoints_file_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value="waypoints.yaml",
        description="Waypoint YAML file",
    )

    map_frame_arg = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="Waypoint frame id",
    )

    takeoff_altitude_arg = DeclareLaunchArgument(
        "takeoff_altitude",
        default_value="5.0",
        description="Takeoff altitude in meters",
    )

    waypoint_manager_node = Node(
        package="waypoint_planner",
        executable="waypoint_manager",
        name="waypoint_manager",
        output="screen",
        parameters=[{
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            "map_frame": LaunchConfiguration("map_frame"),
            "marker_lifetime": 0.0,
        }],
    )

    mission_executor_node = Node(
        package="waypoint_planner",
        executable="mission_executor",
        name="mission_executor",
        output="screen",
        parameters=[{
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            "map_frame": LaunchConfiguration("map_frame"),
            "takeoff_altitude": LaunchConfiguration("takeoff_altitude"),
            "waypoint_arrival_tolerance": 1.0,
            "hover_duration_default": 3.0,
            "max_navigate_timeout": 120.0,
            "control_rate": 20.0,
        }],
    )

    return LaunchDescription([
        waypoints_file_arg,
        map_frame_arg,
        takeoff_altitude_arg,
        waypoint_manager_node,
        mission_executor_node,
    ])
