# 学习者全流程指南

这份文档告诉你作为学习者怎么从零理解代码、怎么运行程序、怎么完成完整盘库流程。

## 1. 先建立全局图

整套系统可以画成：

```text
MID360
  -> FAST-LIO2
  -> /odom_filtered + /cloud_registered

人工/生成航点
  -> mission_executor
  -> /planning/goal_pose
  -> ego_planner_ros2_adapter
  -> EGO-Planner ROS2
  -> /drone_0_planning/pos_cmd
  -> ego_planner_ros2_adapter
  -> /cmd_vel_planned
  -> local_3d_safety_filter
  -> /cmd_vel_safe
  -> px4_cmdvel_bridge
  -> PX4 XRCE-DDS
  -> PX4
```

你先记住一句话：任务层只决定“想怎么飞”，避障层决定“这样飞安不安全”，PX4 桥负责“把安全速度翻译给飞控”。

## 2. 学代码的路线

第一天只读启动和链路：

```text
README.md
launch/auto_flight/bringup_all.launch.py
src/px4_offboard_bridge/launch/offboard_bridge.launch.py
```

第二天读 PX4 DDS：

```text
docs/px4_ros2_dds_guide.md
src/px4_offboard_bridge/src/px4_cmdvel_bridge.cpp
src/px4_offboard_bridge/src/px4_visual_odometry_bridge.cpp
src/px4_offboard_bridge/src/px4_status_bridge.cpp
```

第三天读任务和避障：

```text
src/waypoint_planner/src/mission_executor.cpp
src/px4_offboard_bridge/src/ego_planner_ros2_adapter.cpp
src/px4_offboard_bridge/src/local_3d_safety_filter.cpp
src/waypoint_planner/scripts/generate_inventory_mission.py
```

第四天按 `docs/hands_on_workbook.md` 做实验。

## 3. 怎么用这个程序

完整流程：

1. 飞手手动起飞并飞完整个仓库路线，同时录包。
2. 回放 rosbag，用 FAST-LIO 建图。
3. 保存地图。
4. 设置或生成六仓盘库航点。
5. 启动 XRCE-DDS Agent。
6. 启动全系统。
7. 飞手起飞到安全高度。
8. 切 Offboard 并开始任务。
9. 飞完六仓后自动返回起飞点。

## 4. 离线建图

```bash
ros2 launch px4_mid360 build_map_offline.launch.py \
  bag_path:=./bags/warehouse_mapping \
  map_path:=./maps/warehouse_map.pcd
```

预期：生成 PCD 地图。

## 5. 生成航点

```bash
ros2 run waypoint_planner generate_inventory_mission.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./config/waypoints/six_warehouse_inventory.yaml
```

预期：生成六个并排仓库的“几”字/蛇形盘库航点。

## 6. 启动全系统

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd
```

预期：

- `/odom_filtered` 有定位。
- `/cloud_registered` 有点云。
- `/px4_native/status` 有 PX4 状态。
- EGO-Planner ROS2 发布 `/drone_0_planning/pos_cmd` 后，`/cmd_vel_safe` 可以输出安全速度。

## 7. 启动任务

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/six_warehouse_inventory.yaml \
  takeoff_altitude:=0.0
```

如果由飞手先起飞，`takeoff_altitude:=0.0` 表示任务层不再额外执行起飞爬升。

## 8. 切 Offboard

```bash
ros2 service call /px4_cmdvel_bridge/set_offboard std_srvs/srv/Trigger "{}"
ros2 service call /px4_cmdvel_bridge/arm std_srvs/srv/Trigger "{}"
```

真实飞机上必须保证飞手能随时接管。

## 9. 自动规划怎么用

自动路径规划是技术储备，用于非盘库或需要最优路线的场景。

```bash
ros2 run waypoint_planner optimize_inventory_route.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./maps/map.yaml \
  --output-layout ./config/missions/six_warehouse_layout_optimized.yaml
```

盘库主流程仍建议使用人工确认过的航点，因为仓库环境窄、货架多、飞行风险高。

## 10. 你需要会排查的三类问题

- DDS 问题：看 `/fmu/out/vehicle_status` 和 `/px4_native/status`。
- 定位问题：看 `/odom_filtered` 和 `/cloud_registered`。
- 控制问题：看 `/planning/goal_pose`、`/drone_0_planning/pos_cmd`、`/cmd_vel_planned`、`/cmd_vel_safe`、`/fmu/in/trajectory_setpoint`。
