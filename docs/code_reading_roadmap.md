# 读码路线图

目标：你不是只会运行这个项目，而是能看懂每个节点为什么存在、数据怎么流、哪些地方适合改。

## 1. 先看总入口

先看文件：

```text
launch/auto_flight/bringup_all.launch.py
```

先读函数：

```text
generate_launch_description()
```

为什么先看它：这是整套自动飞行系统的总开关。你会看到 MID360、FAST-LIO、地图加载、重定位、EGO-Planner ROS2、PX4 DDS 桥和安全监控按什么顺序启动。

读完你应该知道：项目主链路是 PX4 XRCE-DDS，不再启动旧的 ROS 飞控桥，也不再把二维地面机器人局部控制器作为默认局部避障。

## 2. 再看 PX4 桥入口

看文件：

```text
src/px4_offboard_bridge/launch/offboard_bridge.launch.py
```

重点看每个 `Node(...)`：

- `ego_planner_path_follower`
- `ego_planner_ros2_adapter`
- `local_3d_safety_filter`
- `px4_visual_odometry_bridge`
- `px4_cmdvel_bridge`
- `px4_status_bridge`

为什么看它：这里决定 ROS 2 任务层如何接入 PX4。

读完你应该能画出这条链：

```text
/planning/goal_pose
  -> ego_planner_ros2_adapter
  -> EGO-Planner ROS2
  -> /drone_0_planning/pos_cmd
  -> ego_planner_ros2_adapter
  -> /cmd_vel_planned
  -> local_3d_safety_filter
  -> /cmd_vel_safe
  -> px4_cmdvel_bridge
  -> /fmu/in/trajectory_setpoint
```

## 3. 看 PX4 参数

看文件：

```text
src/px4_offboard_bridge/config/offboard_params.yaml
config/px4/offboard_params.yaml
```

重点看：

- `px4_cmdvel_bridge` 的速度限制、Offboard 频率、自动解锁开关。
- `px4_visual_odometry_bridge` 的输入/输出话题。
- `px4_status_bridge` 的 `/fmu/out/*` 状态话题。
- `ego_planner_ros2_adapter` 的 EGO 话题适配和 `PositionCommand` 到速度命令的转换。
- `ego_planner_path_follower` 的路径跟随速度限制和前视距离；它是备用接口，用于外部规划器直接发布 `/planning/local_path` 的场景。
- `local_3d_safety_filter` 的点云话题、危险距离、减速距离和安全走廊尺寸。

为什么看它：实机调试时，很多行为不是先改代码，而是先改这些参数。

## 4. 看速度到 PX4 的桥

看文件：

```text
src/px4_offboard_bridge/src/px4_cmdvel_bridge.cpp
```

按这个顺序读：

1. 构造函数。
2. `/cmd_vel_safe` 订阅回调。
3. `publish_setpoint()`。
4. `publish_vehicle_command()`。
5. arm、disarm、set_offboard 服务。

为什么看它：这是自动飞行最后真正把速度送进 PX4 的地方。

重点理解：

- 上游只发 ROS 2 `Twist`。
- 这个节点持续发布 `OffboardControlMode` 和 `TrajectorySetpoint`。
- 它把 ROS ENU/FLU 速度转换成 PX4 NED/FRD 速度。
- 默认不自动解锁、不自动切 Offboard，避免实机误动作。

## 5. 看 FAST-LIO 到 PX4 EKF2 的桥

看文件：

```text
src/px4_offboard_bridge/src/px4_visual_odometry_bridge.cpp
```

按这个顺序读：

1. 构造函数。
2. 里程计订阅回调。
3. 坐标转换函数。
4. `/fmu/in/vehicle_visual_odometry` 发布。

为什么看它：PX4 在室内没有 GPS，必须接收 FAST-LIO 的外部视觉/里程计信息。

重点理解：

- 输入是 `/odom_filtered`。
- 输出是 `/fmu/in/vehicle_visual_odometry`。
- ROS 常用 ENU/FLU，PX4 使用 NED/FRD，需要转换。

## 6. 看 PX4 状态桥和 QoS

看文件：

```text
src/px4_offboard_bridge/src/px4_status_bridge.cpp
```

先看构造函数里的订阅：

- `/fmu/out/vehicle_status`
- `/fmu/out/vehicle_command_ack`

为什么看它：PX4 官方说明 `/fmu/out/*` 订阅不能随便用 ROS 2 默认 QoS。这个文件使用 `rmw_qos_profile_sensor_data`，否则可能“看得到 topic，但收不到数据”。

输出：

- `/px4_native/status`
- `/px4_native/armed`

## 7. 看 EGO-Planner 接入和 3D 安全过滤

看文件：

```text
src/px4_offboard_bridge/src/ego_planner_ros2_adapter.cpp
src/px4_offboard_bridge/src/local_3d_safety_filter.cpp
docs/ego_planner_integration.md
```

先读 `ego_planner_ros2_adapter.cpp`：

1. 构造函数里的参数和话题。
2. `/planning/goal_pose` 到 `/move_base_simple/goal` 的转发。
3. `/odom_filtered` 和 `/cloud_registered` 到 EGO 命名话题的转发。
4. `/drone_0_planning/pos_cmd` 回调。
5. 世界系轨迹命令到机体系 `/cmd_vel_planned` 的转换。

再读 `local_3d_safety_filter.cpp`：

1. 构造函数里的安全参数。
2. 点云回调。
3. 速度回调。
4. 安全走廊检查。
5. 减速/急停输出。

为什么看它：EGO-Planner 才是主要 3D 局部轨迹规划层；`local_3d_safety_filter` 不是规划器，只是最后兜底，发现前方走廊危险时减速或停车。

重点理解：

- 输入 `/planning/goal_pose`、`/odom_filtered`、`/cloud_registered`、`/drone_0_planning/pos_cmd`。
- 输出 `/cmd_vel_planned` 和 `/cmd_vel_safe`。
- safety filter 不再做单独高度逃逸，也不做“自己绕开障碍”的局部规划。

## 8. 看任务执行器

看文件：

```text
src/waypoint_planner/src/mission_executor.cpp
```

按这个顺序读：

1. 构造函数。
2. `handle_start()`。
3. `control_loop()`。
4. `execute_takeoff()`。
5. `execute_navigate()`。
6. `execute_hover()`。
7. `execute_rtl()`。
8. `send_velocity_toward_pose()`。
9. `publish_goal_pose()`。

为什么看它：这是“人工航点盘库”的任务层。它负责按航点推进，并把当前目标发布到 `/planning/goal_pose`，让 EGO-Planner 或适配器生成局部路径。

读完你应该知道：任务层仍然会计算航点距离和任务状态，但真正的 3D 局部避障由 EGO-Planner 侧处理，PX4 控制由下游桥处理。

## 9. 看六仓盘库生成

看文件：

```text
config/missions/six_warehouse_layout.yaml
src/waypoint_planner/scripts/generate_inventory_mission.py
```

重点看：

- 仓库入口、出口、行距、层数配置。
- “几”字/蛇形航线如何生成。
- 六个连续并排仓库如何串起来。

为什么看它：这是把业务需求变成航点文件的地方。

## 10. 看自动规划技术储备

看文件：

```text
src/waypoint_planner/scripts/optimize_inventory_route.py
```

为什么看它：A* 或类似路径搜索不是盘库主流程，但可用于其他场景的最优路线规划。

当前定位：可调用、可扩展、作为技术储备；盘库默认仍走人工/生成航点。

## 11. 最后看安全监控

看目录：

```text
src/safety_monitor/src
```

建议顺序：

1. `communication_watchdog.cpp`
2. `ekf_health_monitor.cpp`
3. `battery_monitor.cpp`
4. `geofence_monitor.cpp`

为什么看它：这些节点负责在真实飞行前发现链路断开、电池低、定位异常、越界等问题。
