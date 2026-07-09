# 六仓室内无人机盘库流程

目标场景：PX4 + MID360 + 机载电脑，飞六个连续并排仓库。飞手先手动采集数据并建图，然后基于地图和人工确认航点执行自动盘库。

## 1. 业务路线

飞行路线：

```text
起飞点
  -> 仓库 1 入口 -> 仓内“几”字/蛇形盘库 -> 仓库 1 出口
  -> 仓库 2 入口 -> 仓内“几”字/蛇形盘库 -> 仓库 2 出口
  -> ...
  -> 仓库 6 出口
  -> 返回起飞点
```

## 2. 建图流程

第一遍由飞手手动飞：

- 从起飞点起飞。
- 依次进出六个仓库。
- 仓内按盘库轨迹飞。
- 全程记录 MID360、IMU、里程计等数据。

然后回放数据：

```bash
ros2 launch px4_mid360 build_map_offline.launch.py \
  bag_path:=./bags/warehouse_mapping \
  map_path:=./maps/warehouse_map.pcd
```

输出：

- FAST-LIO PCD 地图。
- 可选转换为占据栅格地图，供 A* 技术储备脚本使用。

## 3. 航点流程

盘库主流程使用人工确认或脚本生成航点：

```bash
ros2 run waypoint_planner generate_inventory_mission.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./config/waypoints/six_warehouse_inventory.yaml
```

人工确认重点：

- 每个仓库入口和出口是否安全。
- “几”字/蛇形线是否避开货架。
- 转弯半径是否适合无人机。
- 高度是否避开横梁、吊牌、喷淋和灯具。

## 4. 自动飞行流程

启动主系统：

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  enable_ego_planner_ros2:=true
```

启用后，系统会启动 EGO-Planner ROS2 分支和 `ego_planner_ros2_adapter`。

启动任务：

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/six_warehouse_inventory.yaml \
  takeoff_altitude:=0.0
```

控制链：

```text
mission_executor
  -> /planning/goal_pose
  -> ego_planner_ros2_adapter
  -> EGO-Planner ROS2
  -> /drone_0_planning/pos_cmd
  -> ego_planner_ros2_adapter
  -> /cmd_vel_planned
  -> local_3d_safety_filter
  -> /cmd_vel_safe
  -> px4_cmdvel_bridge
  -> /fmu/in/trajectory_setpoint
  -> PX4
```

## 5. 3D 局部避障定位

EGO-Planner ROS2 使用适配后的 `/drone_0_cloud_registered` 和 `/drone_0_odom_filtered` 做 3D 局部轨迹规划，并向本项目输出 `/drone_0_planning/pos_cmd`。

`local_3d_safety_filter` 是最后安全层，不是规划器。它只检查当前速度方向上的点云安全走廊：安全则放行，接近障碍则减速，进入危险距离则停车。

默认配置 `pointcloud_frame_mode: world` 适配 FAST-LIO 注册点云：节点会用 `/odom_filtered` 把世界系点云转到无人机机体系，再判断前方、左右、上下障碍。如果以后接入已经在机体系下的点云，再把该参数改成 `body`。

## 6. 自动规划技术储备

A* 或类似规划能力保留在项目中，用于其他场景：

```bash
ros2 run waypoint_planner optimize_inventory_route.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./maps/map.yaml \
  --output-layout ./config/missions/six_warehouse_layout_optimized.yaml
```

当前建议：

- 六仓盘库默认使用人工确认航点。
- 自动规划用于路线优化、其他仓型，或后续扩展。

## 7. 当前能力判断

当前项目目标状态：

- 可离线建图。
- 可选择在线建图。
- 可生成六仓盘库航点。
- 可通过 EGO-Planner 做 3D 局部轨迹规划。
- 可通过 safety filter 做最后减速/停车保护。
- 可通过 PX4 XRCE-DDS 发送 Offboard setpoint。
- 可读取 PX4 DDS 状态和电池信息。
- 可保留 A* 自动规划作为技术储备。

实机前仍必须完成：

- 坐标转换低速验证。
- Offboard 切换和遥控器接管验证。
- EGO-Planner 输出路径连续性验证。
- 单仓低速测试。
- 双仓连续测试。
- 最后再做六仓连续任务。
