# 实操手册

这份手册按实验步骤写。每一步都有命令和预期现象，适合你像做实验一样验证整套程序。

## 实验 0：编译

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

预期现象：

- 编译通过。
- `ros2 pkg list | grep px4_offboard_bridge` 能看到包。

## 实验 1：确认 PX4 DDS 通了

先启动 XRCE-DDS Agent：

```bash
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
```

另开终端：

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep /fmu
ros2 topic echo /fmu/out/vehicle_status --once
```

预期现象：

- 能看到 `/fmu/in/...` 和 `/fmu/out/...`。
- `vehicle_status` 能输出一次。

如果失败：

- 先检查 PX4 `uxrce_dds_client`。
- 再检查 Agent 串口/UDP。
- 最后检查 ROS domain 和 QoS。

## 实验 2：确认 MID360

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

预期现象：

- 两个话题都有稳定频率。
- 移动雷达时点云变化正常。

## 实验 3：确认 FAST-LIO 输出

```bash
ros2 topic echo /odom_filtered --once
ros2 topic hz /cloud_registered
```

预期现象：

- `/odom_filtered` 有位置和姿态。
- `/cloud_registered` 有注册点云。

小实验：

- 人拿着飞机向前平移 0.5 m。
- 观察 odom 方向是否和实际一致。

## 实验 4：启动 PX4 DDS 桥

```bash
ros2 launch px4_offboard_bridge offboard_bridge.launch.py
```

另开终端检查：

```bash
ros2 topic echo /px4_native/status --once
ros2 topic echo /fmu/in/vehicle_visual_odometry --once
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/in/trajectory_setpoint
```

预期现象：

- `/px4_native/status` 有输出。
- `/fmu/in/vehicle_visual_odometry` 有输出。
- Offboard 心跳和 setpoint 持续发布。

## 实验 5：手动发速度，不解锁测试

```bash
ros2 topic pub /cmd_vel_planned geometry_msgs/msg/Twist \
"{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 5
```

另开终端：

```bash
ros2 topic echo /cmd_vel_safe --once
ros2 topic echo /fmu/in/trajectory_setpoint --once
```

预期现象：

- `/cmd_vel_safe` 有速度输出。
- `/fmu/in/trajectory_setpoint` 有 PX4 setpoint。
- 此时不解锁，飞机不会飞，只验证链路。

## 实验 6：验证 3D 局部避障

保持实验 5 的小速度发布，然后在雷达前方安全距离内放入障碍物。

```bash
ros2 topic echo /cmd_vel_safe
```

预期现象：

- 无障碍时，`/cmd_vel_safe` 接近 `/cmd_vel_planned`。
- 前方有障碍时，前进速度下降。
- EGO-Planner ROS2 正常时，`/drone_0_planning/pos_cmd` 会更新，`/cmd_vel_planned` 会跟随局部轨迹命令；安全过滤器只在危险时减速或刹停。

## 实验 7：离线建图

```bash
ros2 launch px4_mid360 build_map_offline.launch.py \
  bag_path:=./bags/warehouse_mapping \
  map_path:=./maps/warehouse_map.pcd
```

预期现象：

- FAST-LIO 回放 bag。
- 地图点云逐步积累。
- 最终生成 `warehouse_map.pcd`。

## 实验 8：生成六仓盘库航点

```bash
ros2 run waypoint_planner generate_inventory_mission.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./config/waypoints/six_warehouse_inventory.yaml
```

预期现象：

- 生成六个连续仓库的盘库航点。
- 每个仓库内部是类似“几”字/蛇形飞行。
- 最后包含返回起飞点的任务逻辑。

## 实验 9：启动全系统

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  enable_ego_planner_ros2:=true
```

检查：

```bash
ros2 topic echo /px4_native/status --once
ros2 topic echo /odom_filtered --once
ros2 topic echo /drone_0_planning/pos_cmd --once
ros2 topic echo /cmd_vel_safe --once
```

预期现象：

- PX4 状态、定位、EGO 轨迹命令和安全速度输出都正常。

## 实验 10：启动任务

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/six_warehouse_inventory.yaml \
  takeoff_altitude:=0.0
```

预期现象：

- 任务执行器加载航点。
- `/mission/start` 被调用后任务开始。
- `/planning/goal_pose` 输出当前任务目标。
- EGO-Planner ROS2 输出 `/drone_0_planning/pos_cmd`。
- `ego_planner_ros2_adapter` 转成 `/cmd_vel_planned`，`local_3d_safety_filter` 做最后安全检查。
- PX4 DDS 桥持续发送 setpoint。

## 实验 11：切 Offboard 和解锁

只在安全、拆桨或有保护措施时执行：

```bash
ros2 service call /px4_cmdvel_bridge/set_offboard std_srvs/srv/Trigger "{}"
ros2 service call /px4_cmdvel_bridge/arm std_srvs/srv/Trigger "{}"
```

预期现象：

- PX4 进入 Offboard。
- `/px4_native/armed` 变为 true。

停止：

```bash
ros2 service call /px4_cmdvel_bridge/disarm std_srvs/srv/Trigger "{}"
```

## 实验 12：自动规划技术储备

```bash
ros2 run waypoint_planner optimize_inventory_route.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./maps/map.yaml \
  --output-layout ./config/missions/six_warehouse_layout_optimized.yaml
```

预期现象：

- 基于栅格地图优化访问顺序或路径代价。
- 这个功能用于其他场景技术储备，盘库主流程不强制依赖它。
