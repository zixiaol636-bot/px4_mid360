#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    use_ego_planner_arg = DeclareLaunchArgument(
        "use_ego_planner",
        default_value="true",
        description="true: mission publishes goals for EGO-Planner; false: direct velocity fallback",
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
            "takeoff_altitude": ParameterValue(
                LaunchConfiguration("takeoff_altitude"), value_type=float),
            "waypoint_arrival_tolerance": 1.0,
            "hover_duration_default": 3.0,
            "max_navigate_timeout": 120.0,
            "control_rate": 20.0,
            "goal_pose_topic": "/planning/goal_pose",
            "publish_goal_pose": True,
            "publish_cmd_vel": ParameterValue(
                PythonExpression([
                    "'False' if '",
                    LaunchConfiguration("use_ego_planner"),
                    "'.lower() in ('1', 'true', 'yes', 'on') else 'True'",
                ]),
                value_type=bool,
            ),
            "cmd_vel_topic": ParameterValue(
                PythonExpression([
                    "'/mission/direct_cmd_vel_unused' if '",
                    LaunchConfiguration("use_ego_planner"),
                    "'.lower() in ('1', 'true', 'yes', 'on') else '/cmd_vel_planned'",
                ]),
                value_type=str,
            ),
        }],
    )

    return LaunchDescription([
        waypoints_file_arg,
        map_frame_arg,
        takeoff_altitude_arg,
        use_ego_planner_arg,
        waypoint_manager_node,
        mission_executor_node,
    ])
