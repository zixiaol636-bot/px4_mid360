#!/usr/bin/env python3
"""
MID360 驱动 + 看门狗 启动文件

启动两件事：
1. livox_ros_driver2 — 连接 MID360 硬件，发布 /livox/lidar (10Hz) 和 /livox/imu (200Hz)
2. driver_monitor  — 监控点云是否断流，超时告警
"""

import os

from ament_index_python.packages import get_package_share_directory  # 找包安装路径
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit          # 监听进程退出事件
from launch.events import Shutdown                      # 触发整个 launch 关闭
from launch.substitutions import LaunchConfiguration    # 在运行时解析参数值
from launch_ros.actions import Node


def generate_launch_description():
    # 找到本包安装后的路径，拼接默认配置文件位置
    pkg_dir = get_package_share_directory("mid360_driver_wrapper")
    default_config_path = os.path.join(pkg_dir, "config", "MID360_config.json")

    # ---------- 启动参数 ----------
    config_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="MID360_config.json 的绝对路径",
    )

    lidar_topic_arg = DeclareLaunchArgument(
        "lidar_topic",
        default_value="/livox/lidar",
        description="livox_ros_driver2 发布的点云话题名",
    )

    # ---------- livox_ros_driver2 官方驱动 ----------
    livox_node = Node(
        package="livox_ros_driver2",              # 官方包
        executable="livox_ros_driver2_node",       # 可执行文件名
        name="livox_ros_driver2",
        output="screen",                           # 日志打到终端
        parameters=[{
            "xfer_format": 0,       # 0 = PointCloud2 格式（ROS 标准）
            "multi_topic": 0,       # 0 = 单 topic 模式
            "data_src": 0,          # 0 = 真实雷达（非回放 bag）
            "publish_freq": 10.0,   # 点云发布频率 [Hz]
            "output_data_type": 0,  # 0 = 点云数据
        }],
        arguments=[
            "--ros-args",
            "--params-file",
            LaunchConfiguration("config_path"),     # 加载 MID360_config.json（IP/端口）
        ],
    )

    # ---------- driver_monitor 看门狗 ----------
    driver_monitor_node = Node(
        package="mid360_driver_wrapper",
        executable="driver_monitor",
        name="driver_monitor",
        output="screen",
        parameters=[{
            "lidar_topic": LaunchConfiguration("lidar_topic"),  # 监控哪个话题
            "watchdog_timeout": 2.0,    # 超过 2 秒没收到点云 → 告警
            "check_rate": 10.0,         # 每秒检查 10 次
            "publish_health": True,     # 在 /driver/health 发布健康状态
        }],
    )

    # ---------- 退出事件处理 ----------
    # 如果 livox_ros_driver2 进程退出（崩溃/被 kill），自动关闭整个 launch
    exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=livox_node,           # 监听这个节点的退出
            on_exit=[
                LogInfo(msg="livox_ros_driver2 已退出，关闭整个 launch."),
                EmitEvent(event=Shutdown()),    # 触发关闭，所有节点一起退出
            ],
        )
    )

    return LaunchDescription([
        config_arg,           # 声明参数，命令行可覆盖
        lidar_topic_arg,
        livox_node,           # 启动驱动
        driver_monitor_node,  # 启动看门狗
        exit_handler,         # 注册退出监听
    ])
