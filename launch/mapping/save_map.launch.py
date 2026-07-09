#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="./maps/warehouse_map.pcd",
        description="PCD file path produced by map_saver",
    )

    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="./maps",
        description="Directory for generated PGM/YAML outputs",
    )

    resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="0.1",
        description="PGM resolution in meters per cell",
    )

    save_delay_arg = DeclareLaunchArgument(
        "save_delay",
        default_value="1.0",
        description="Delay before triggering /map/save",
    )

    convert_delay_arg = DeclareLaunchArgument(
        "convert_delay",
        default_value="3.0",
        description="Delay before converting PCD to PGM/YAML",
    )

    save_pcd = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/map/save",
            "std_srvs/srv/Trigger",
            "{}",
        ],
        output="screen",
    )

    convert_pgm = Node(
        package="map_manager",
        executable="pcd_to_pgm",
        name="pcd_to_pgm",
        output="screen",
        parameters=[{
            "input_pcd": LaunchConfiguration("map_path"),
            "output_dir": LaunchConfiguration("output_dir"),
            "resolution": ParameterValue(LaunchConfiguration("resolution"), value_type=float),
            "z_min": -0.5,
            "z_max": 3.5,
            "auto_run": True,
        }],
    )

    return LaunchDescription([
        map_path_arg,
        output_dir_arg,
        resolution_arg,
        save_delay_arg,
        convert_delay_arg,
        LogInfo(msg="=== Triggering map save; map_saver must already be running ==="),
        TimerAction(period=LaunchConfiguration("save_delay"), actions=[save_pcd]),
        TimerAction(period=LaunchConfiguration("convert_delay"), actions=[convert_pgm]),
    ])
