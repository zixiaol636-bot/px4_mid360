# 学习指南

这份文档不是“泛泛介绍 ROS/PX4”，而是围绕当前仓库已经整理后的真实架构，帮助你快速看懂每个包在系统里负责什么，以及后续应该怎么继续改。

## 1. 这套系统到底在做什么

目标是做一套室内仓储无人机系统：

- MID360 负责感知
- FAST-LIO2 负责里程计 / 建图 / 定位
- MAVROS 负责 PX4 桥接
- Nav2 或任务节点负责给速度指令
- 安全监控负责兜底

当前代码里，真正的控制主链已经被整理成：

```text
/livox/lidar + /livox/imu
        -> FAST-LIO2
        -> /Odometry
        -> fast_lio_bridge/odometry_relay
        -> /odom_filtered
        -> vision_pose_bridge
        -> /mavros/vision_pose/pose
        -> PX4 / EKF2

/cmd_vel
        -> local_3d_avoidance
        -> /cmd_vel_safe
        -> cmdvel_to_setpoint
        -> /mavros/setpoint_velocity/cmd_vel
        -> PX4 Offboard
```

这条链路的好处是：

- 任务节点、Nav2、临时测试节点都可以统一往 `/cmd_vel` 发命令
- 3D 局部避障、速度限制和平滑被拆成独立层，职责更清楚

## 2. 仓库结构怎么理解

### `src/warehouse_utils`

公共工具层。

现在主要包含：

- `yaml_utils`
- `frame_conversions`

实际价值：

- 航点 YAML 的读写逻辑集中在一个地方
- 避免每个包都手写一套 waypoint 文件解析

### `src/fast_lio_bridge`

把 FAST-LIO2 输出整理成下游更容易消费的格式。

包含：

- `odometry_relay`
- `pointcloud_relay`

`odometry_relay` 当前职责：

- 订阅 `/Odometry`
- 做轻量平滑
- 输出 `/odom_filtered`
- 可选发布 `odom -> base_link` TF

### `src/map_manager`

地图管理层。

包含：

- `map_saver`
- `map_loader`
- `pcd_to_pgm`

建议把它理解成三件独立的事：

1. 累积点云并保存成 `.pcd`
2. 把 `.pcd` 加载成 `/map_cloud`
3. 把 `.pcd` 转成 Nav2 用的 `map.pgm + map.yaml`

### `src/relocalizer`

ICP 重定位。

当前模式：

- 订阅 `/map_cloud`
- 订阅 `/cloud_registered`
- 服务触发或持续修正
- 发布 `map -> odom` TF

这个包是“定位闭环”里的关键点。

### `src/px4_offboard_bridge`

PX4 桥接层。

当前最关键的节点有三个：

- `vision_pose_bridge`
- `local_3d_avoidance`
- `cmdvel_to_setpoint`
- `offboard_controller`

你可以把它们拆开理解：

`vision_pose_bridge`

- 把 `/odom_filtered` 转发给 MAVROS 视觉位姿输入

`cmdvel_to_setpoint`

- 把 `/cmd_vel_safe` 平滑成 MAVROS 可用速度指令
- 做速度上限和加速度限制

`local_3d_avoidance`

- 订阅 `/cmd_vel`、`/cloud_registered`、`/odom_filtered`
- 输出 `/cmd_vel_safe`
- 基于点云做反应式 3D 局部避障
- 当前它才是“人工航点 + 3D 局部避障”目标的核心节点

`offboard_controller`

- 这是一个演示性质的起飞/悬停/降落状态机
- 现在默认不会自动启动，避免和真实任务节点抢控制权

### `src/safety_monitor`

安全监控层。

包含：

- `battery_monitor`
- `communication_watchdog`
- `ekf_health_monitor`
- `geofence_monitor`

这个包当前更偏“状态感知 + 轻量兜底”，不是完整的 flight termination system。

### `src/waypoint_planner`

任务层。

包含：

- `waypoint_manager`
- `mission_executor`

`waypoint_manager`

- 航点文件读写
- 航点 Marker 发布
- 启动时自动加载当前 YAML，方便直接在 RViz 检查保存过的航点

`mission_executor`

- 加载 YAML 航点
- 起飞
- 顺序导航
- 悬停
- RTL

这部分是从示例骨架里重点补强过的：

- 开始任务前会检查 odom 是否到位
- 会记录 home pose
- RTL 不再是“原地下落”，而是先回起飞点再下降
- 统一改成往 `/cmd_vel` 发命令

## 3. 当前系统的几个关键设计判断

### 判断 1：统一控制出口

这是这轮整理里最重要的改动之一。

之前不同节点在抢不同的 MAVROS 话题：

- 有的发 `/mavros/setpoint_raw/local`
- 有的发 `/mavros/setpoint_velocity/cmd_vel`
- 有的直接内部做坐标变换

这样系统很容易互相覆盖。

现在的建议是：

- 上层统一发 `/cmd_vel`
- 下游先由 `local_3d_avoidance` 做 3D 避障，再由 `cmdvel_to_setpoint` 转成飞控速度指令

### 判断 2：不要让演示控制器默认接管系统

以前 `offboard_controller` 会和任务层抢控制。

现在：

- launch 里默认不启动它
- 即使启动，默认也不自动起飞

这让 `bringup_all.launch.py` 更适合作为“基础系统启动”，不是“上电就起飞”。

### 判断 3：地图流程要固定产物路径

离线建图最怕的不是算法，而是产物对不上。

这次把：

- `build_map_offline.launch.py`
- `map_saver`
- `save_map.launch.py`

都围绕同一个 `map_path` 打通了，避免前一步输出随机文件名、后一步又找不到。

## 4. 你以后想继续优化，先从哪几块入手

### 方向 A：把任务层接到 Nav2 Goal，而不是直接发速度

当前 `mission_executor` 还是“自己算速度，自己发 `/cmd_vel`”。

这能用，但长期来看更好的方向是：

- mission 节点只负责状态机和航点管理
- 真正的路径规划和避障交给 Nav2
- mission 节点给 Nav2 发导航目标

什么时候值得做：

- 你需要复杂避障
- 你希望仓库货架之间自动绕行
- 你想减少手写速度控制逻辑

### 方向 B：把安全监控升级成真正的仲裁层

当前安全监控主要是告警和轻量介入。

更工程化的方向是做一个 command arbiter：

- mission / nav2 输出一份命令
- safety monitor 输出一份约束
- arbiter 决定最终允许下发什么速度

这样比多个节点同时往控制话题发消息更稳。

### 方向 C：把 SITL 测试补上

这套仓库现在最缺的不是新功能，而是自动化验证。

建议最先补：

- 最小 launch 冒烟测试
- YAML 航点加载测试
- 地图转换测试
- mission 状态机单元测试

## 5. 推荐的学习顺序

如果你要真正吃透这个仓库，建议按下面顺序读：

1. `README.md`
2. `launch/auto_flight/bringup_all.launch.py`
3. `src/px4_offboard_bridge/config/offboard_params.yaml`
4. `src/waypoint_planner/src/mission_executor.cpp`
5. `src/fast_lio_bridge/src/odometry_relay.cpp`
6. `src/map_manager/src/map_saver.cpp`
7. `src/relocalizer/src/icp_relocalizer.cpp`

## 6. 一条最小可工作的验证路径

### 只验证定位

```bash
ros2 launch px4_mid360 localize_only.launch.py \
  map_path:=./maps/warehouse_map.pcd
```

看这些是否正常：

- `/odom_filtered`
- `/map_cloud`
- `/tf`

### 验证全系统但不飞

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  map_grid_path:=./maps/map.yaml
```

重点看：

- `/mavros/state`
- `/mavros/vision_pose/pose`
- `/cmd_vel`
- `/mavros/setpoint_velocity/cmd_vel`

### 跑任务

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/warehouse_example.yaml
```

## 7. 这套仓库现在适合怎么用

适合：

- 做室内无人机系统原型
- 做仓储巡检 / 盘库场景 PoC
- 作为你后续继续工程化的底座

不适合直接当成：

- 已经过完整飞行安全验证的生产系统
- 无需调参即可上场的商业成品

## 8. 最后给你的建议

如果你后面继续沿这个项目做，不要再回到“让模型一次性生成整个仓库”的模式了。更稳的方式是：

1. 先定一条最小主链。
2. 只让新改动影响一层。
3. 每加一层就补一层验证。

这轮我已经把仓库从“像项目”往“能继续做成项目”推了一大步，后面最值钱的事情会是测试和真机联调，而不是继续堆节点数量。
