# PX4 + MID360 项目搭建指南

这份指南对应当前仓库已经整理过的实际工作流，目标是把环境、建图、定位、任务控制和盘库任务串成一条能落地联调的链路。

## 1. 目标环境

- Ubuntu 22.04
- ROS 2 Humble
- PX4 飞控
- MAVROS ROS 2 版本
- Livox MID360
- FAST-LIO2

建议工作空间结构：

```bash
~/ros2_ws/
  src/
    px4_mid360/
```

## 2. 拉取仓库和依赖

```bash
cd ~/ros2_ws/src
git clone <your-repo> px4_mid360
vcs import . < px4_mid360/dependencies.repos
```

如果你已经有自己的 `mavros2`、`fast_lio2`、`nav2_bringup` 版本，也可以不重复拉取，但要自己保证版本兼容。

## 3. 系统依赖

至少安装这些常见依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-rosdep2 \
  libpcl-dev \
  libyaml-cpp-dev \
  libeigen3-dev \
  ros-humble-pcl-conversions \
  ros-humble-tf2-geometry-msgs
```

然后执行：

```bash
sudo rosdep init || true
rosdep update
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 4. 编译

```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

仓库顶层现在已经是标准 ROS 2 包，所以可以直接使用：

```bash
ros2 launch px4_mid360 bringup_all.launch.py
```

## 5. 传感器和飞控连接

### MID360

- 推荐固定 IP
- 常见配置示例：
  - 机载电脑：`192.168.1.50`
  - MID360：`192.168.1.100`

```bash
sudo ip addr add 192.168.1.50/24 dev eth0
ping 192.168.1.100
```

### PX4

- 使用 USB、串口或网络方式接入机载电脑
- 确保 MAVROS 能连到飞控
- 建议先在 QGroundControl 里完成基础校准

## 6. 离线建图流程

### 第一步：录包

```bash
ros2 launch px4_mid360 record_bag.launch.py
```

手动飞完整个目标区域，至少覆盖：

- 起飞区
- 六个仓的入口
- 仓内主要通道
- 仓间过渡区域

### 第二步：离线回放建图

```bash
ros2 launch px4_mid360 build_map_offline.launch.py \
  bag_path:=./bags/warehouse_mapping \
  map_path:=./maps/warehouse_map.pcd
```

这一步会启动：

- FAST-LIO2
- `fast_lio_bridge/odometry_relay`
- `map_manager/map_saver`
- `ros2 bag play`

### 第三步：保存并转成 Nav2 栅格图

```bash
ros2 launch px4_mid360 save_map.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  output_dir:=./maps
```

产物通常包括：

- `warehouse_map.pcd`
- `map.pgm`
- `map.yaml`

## 7. 可选模式：在线建图

如果你想做“飞行时同步建图”，现在项目里已经补了统一入口：

```bash
ros2 launch px4_mid360 online_mapping.launch.py \
  map_path:=./maps/warehouse_online_map.pcd
```

这条链会启动：

- MID360 驱动
- MAVROS
- FAST-LIO2 mapping
- `fast_lio_bridge`
- `map_manager/map_saver`

它更适合：

- 前期新场地快速采图
- 在线建图方案验证
- 现场试扫和技术储备

正式盘库仍建议优先使用“先建好图，再自动执行任务”的离线路径，风险更可控。

## 8. 定位与全系统启动

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  map_grid_path:=./maps/map.yaml
```

这会依次启动：

- MID360 驱动包装
- MAVROS
- FAST-LIO2 定位模式
- 点云和里程计桥接
- 地图加载
- ICP 重定位
- Nav2
- offboard 桥接
- 安全监控

说明：

- `offboard_controller` 默认不会自动起飞，避免和任务执行器抢控制权。
- 当前控制链是 `/cmd_vel -> local_3d_avoidance -> /cmd_vel_safe -> cmdvel_to_setpoint`。
- `z_axis_monitor` 现在是可选旧模块，默认关闭，避免和 3D 避障层重复介入。

## 9. 航点任务

示例航点文件：

`config/waypoints/warehouse_example.yaml`

启动任务：

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/warehouse_example.yaml
```

任务启动后会自动调用 `/mission/start`。

如果是“飞手先手动起飞，再切自动”，建议追加：

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/warehouse_example.yaml \
  takeoff_altitude:=0.0
```

## 10. RViz 点击式航点辅助

仓库里保留了一个轻量辅助脚本：

`src/waypoint_planner/scripts/interactive_waypoints.py`

推荐这样启动：

```bash
ros2 run waypoint_planner interactive_waypoints.py --ros-args \
  -p waypoints_file:=./config/waypoints/warehouse_example.yaml
```

使用方式：

- 在 RViz 里使用 `Publish Point`
- 往 `/clicked_point` 点击位置
- 节点把点保存成航点并发布 Marker
- 退出时自动落盘

## 11. 六仓盘库任务生成

如果你的业务是“六个连续并排仓库盘库”，先编辑：

`config/missions/six_warehouse_layout.yaml`

它描述的是：

- 每个仓库的中心点
- 仓长
- 仓宽
- 朝向角
- 盘库高度
- 扫线间距
- 离墙安全余量

### 直接生成六仓任务

```bash
ros2 run waypoint_planner generate_inventory_mission.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./config/waypoints/six_warehouse_inventory.yaml
```

这个脚本会生成每个仓内类似“几”字形的往返盘库轨迹。

### 先优化访问顺序，再生成任务

```bash
ros2 run waypoint_planner optimize_inventory_route.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./maps/map.yaml \
  --output-layout ./config/missions/six_warehouse_layout_optimized.yaml
```

然后再生成：

```bash
ros2 run waypoint_planner generate_inventory_mission.py \
  ./config/missions/six_warehouse_layout_optimized.yaml \
  ./config/waypoints/six_warehouse_inventory.yaml
```

说明：

- 六仓盘库主链是“生成航点 + mission_executor + 3D 局部避障”
- Nav2 的 A* + DWB 是保留的自动规划储备，更适合其他点到点自主导航场景
- 任务完成后，`mission_executor` 会自动进入 RTL，回到任务启动时记录的 home pose

## 12. 建议的上机调试顺序

1. 单独确认 MID360 数据正常。
2. 单独确认 MAVROS 已连接 PX4。
3. 跑 `localize_only.launch.py` 验证地图加载和重定位。
4. 跑 `bringup_all.launch.py` 验证全链路。
5. 生成六仓任务或导入人工航点。
6. 最后再执行自动盘库任务。

## 13. 当前项目的真实定位

这套仓库现在已经从“AI 一次性生成骨架”整理成“可继续工程化”的版本，但仍然建议按工程项目方式使用：

- 先做 SITL 或绑桨测试
- 再做低速实机验证
- 再逐步放开自动任务

不要把它直接当成“零验证即可正式盘库”的成品。
