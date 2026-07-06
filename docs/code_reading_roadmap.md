# 读码路线图

这份文档不是功能说明，而是“怎么读这个仓库”的路线图。

目标是帮你少走弯路：

1. 先看哪些文件
2. 每个文件先看哪个函数
3. 为什么先看它
4. 看完之后你脑子里应该留下什么结论

## 1. 总体阅读原则

读这个仓库，不要上来就从 `src/` 里随机挑一个 C++ 文件硬啃。

更好的顺序是：

1. 先看启动文件
2. 再看参数文件
3. 再看节点源码
4. 最后才看工具层和细节函数

原因很简单：

- launch 决定“系统怎么拼”
- yaml 决定“系统怎么跑”
- 节点源码决定“系统到底做了什么”

## 2. 第一站：系统怎么被拉起来

### 文件

`launch/auto_flight/bringup_all.launch.py`

### 先看什么

先看 `generate_launch_description()`

### 为什么先看它

因为这是全系统入口。

你应该先回答这几个问题：

1. 它一共 include 了哪些包
2. 它按什么顺序启动节点
3. 哪些参数是顶层传下去的

### 看完你应该知道

- 系统不是一个包，而是一组 ROS 2 包拼起来的
- 定位、地图、桥接、Nav2、安全监控是分层启动的

## 3. 第二站：PX4 桥接层

### 文件

`src/px4_offboard_bridge/launch/offboard_bridge.launch.py`

### 先看什么

先看每个 `Node(...)`

重点看：

- `vision_pose_bridge`
- `local_3d_avoidance`
- `cmdvel_to_setpoint`

### 为什么先看它

因为这层决定了：

- 位姿怎么送给 PX4
- 期望速度怎么先经过 3D 避障
- 最终怎么送到 MAVROS/PX4

### 看完你应该知道

当前控制链已经不是：

```text
/cmd_vel -> cmdvel_to_setpoint -> PX4
```

而是：

```text
/cmd_vel -> local_3d_avoidance -> /cmd_vel_safe
         -> cmdvel_to_setpoint -> /mavros/setpoint_velocity/cmd_vel
         -> PX4
```

## 4. 第三站：桥接层参数

### 文件

`src/px4_offboard_bridge/config/offboard_params.yaml`

### 先看什么

按节点块看：

1. `vision_pose_bridge`
2. `local_3d_avoidance`
3. `cmdvel_to_setpoint`
4. `z_axis_monitor`

### 为什么先看它

因为你以后调行为，第一入口基本都在这里。

### 重点参数解释

#### `local_3d_avoidance`

- `cmd_vel_in_topic`
  说明谁在提供期望速度

- `cmd_vel_out_topic`
  说明避障后的安全速度发到哪里

- `forward_influence_distance`
  说明前方多远开始介入避障

- `side_influence_distance`
  说明左右多宽范围被视为局部障碍

- `vertical_influence_distance`
  说明上下多高范围参与避障

- `safety_distance_xy`
  表示前向/水平安全距离阈值

- `hard_stop_distance`
  表示太近时直接强制削弱前进速度

- `repulsion_gain_xy`
  表示横向避让力度

- `repulsion_gain_z`
  表示上下避让力度

- `lateral_escape_gain`
  表示当前方拥堵时，向左右空旷侧偏转的力度

- `vertical_escape_gain`
  表示当前方拥堵时，向上下空旷侧偏转的力度

#### `cmdvel_to_setpoint`

- `cmd_vel_topic`
  现在应该订阅 `/cmd_vel_safe`

- `acceleration_limit`
  决定安全速度送给飞控前的平滑程度

### 看完你应该知道

- 真正“3D 局部避障”的主参数都在 `local_3d_avoidance`
- `cmdvel_to_setpoint` 已经不是避障器，而是末端限速/平滑桥

## 5. 第四站：任务层

### 文件

`src/waypoint_planner/src/mission_executor.cpp`

### 先看什么

按下面顺序看：

1. 构造函数
2. `handle_start()`
3. `control_loop()`
4. `execute_takeoff()`
5. `execute_navigate()`
6. `execute_hover()`
7. `execute_rtl()`
8. `send_velocity_toward_pose()`

### 为什么这么看

因为这基本就是状态机的执行顺序。

### 每个函数你要看懂什么

#### 构造函数

看：

- 订阅了什么
- 发布了什么
- 提供了什么服务
- 定时器多久跑一次

关键结论：

- 它订阅 `/odom_filtered`
- 它输出期望速度到 `/cmd_vel`
- 它不是直接碰 MAVROS 的节点

#### `handle_start()`

看：

- 启动任务前检查了什么
- home pose 在哪记录

关键结论：

- 没有 odom 不让起任务
- 航点从 YAML 加载
- home pose 是任务开始时记录的

#### `control_loop()`

看：

- 当前状态如何切换分支
- 在什么状态下发零速度

关键结论：

- 整个任务节点就是一个定时驱动的状态机

#### `execute_navigate()`

这是最值得看的函数之一。

看：

- 航点到达是怎么判定的
- 超时是怎么处理的
- 最终怎么发运动命令

关键结论：

- 当前任务层依然是“自己朝航点生成期望速度”
- 真正的局部避障已经从这里下沉到了 `local_3d_avoidance`

#### `send_velocity_toward_pose()`

看：

- `dx dy dz` 怎么转成速度
- 为什么还要做世界系到机体系转换

关键结论：

- 任务层输出的是“机体系期望速度”
- 避障层和 PX4 桥接层都在这个输出之后继续处理

## 6. 第五站：3D 局部避障核心

### 文件

`src/px4_offboard_bridge/src/local_3d_avoidance.cpp`

### 先看什么

按下面顺序看：

1. 构造函数
2. `publish_avoided_cmd()`
3. `analyze_obstacles()`
4. `apply_avoidance()`
5. `limit_output()`
6. `smooth_output()`

### 为什么它是关键文件

因为这是这次“人工航点 + 3D 局部避障”目标的核心落点。

### 每个函数要看懂什么

#### 构造函数

看：

- 输入速度话题
- 输出安全速度话题
- 点云和 odom 订阅

关键结论：

- 它是插在任务层和飞控桥之间的“安全层”

#### `publish_avoided_cmd()`

看：

- 什么时候直接透传速度
- 什么时候启动避障
- 最终输出如何平滑

关键结论：

- 没有新速度时会收敛到零
- 没有新点云时会退化成透传/限幅行为

#### `analyze_obstacles()`

这是整个避障算法最关键的函数。

看：

- 点云是在什么坐标系里理解的
- 如何把点云变成机体系障碍分布
- 如何计算前方净空、左右拥堵、上下拥堵

关键结论：

- 它不是全局规划
- 它是基于点云的局部反应式 3D 避障

#### `apply_avoidance()`

看：

- 排斥向量如何叠加到期望速度
- 什么时候强制削弱前进速度
- 什么时候触发侧向/垂向逃逸偏置

关键结论：

- 前方太近时不只是减速
- 还会尝试往更空的左右或上下方向绕

## 7. 第六站：最终给 PX4 的速度长什么样

### 文件

`src/px4_offboard_bridge/src/cmdvel_to_setpoint.cpp`

### 先看什么

1. 构造函数
2. `cmd_vel_callback()`
3. `publish_smoothed_setpoint()`

### 为什么现在仍然要看它

因为避障做完以后，还要经过这层再进入 MAVROS。

### 关键结论

- 这里负责的是“平滑、限幅、超时归零”
- 它不再是主要避障逻辑所在

## 8. 第七站：位姿输入 PX4

### 文件

`src/px4_offboard_bridge/src/vision_pose_bridge.cpp`

### 先看什么

1. 构造函数
2. `odom_callback()`
3. `publish_latest_pose()`

### 为什么要看

因为无人机飞得稳不稳，根子上取决于 PX4 有没有吃到正确的外部位姿。

### 关键结论

- `/odom_filtered` 是你本项目里非常关键的桥梁话题

## 9. 第八站：点云和里程计桥

### 文件

`src/fast_lio_bridge/src/odometry_relay.cpp`
`src/fast_lio_bridge/src/pointcloud_relay.cpp`

### 怎么看

先看 `odometry_relay.cpp`，后看 `pointcloud_relay.cpp`

#### `odometry_relay.cpp`

先看：

- 构造函数
- `odom_callback()`
- TF 发布部分

关键结论：

- `/Odometry` 被整理成更统一的 `/odom_filtered`

#### `pointcloud_relay.cpp`

先看：

- 输入输出 topic
- 是否改 frame
- 是否下采样

关键结论：

- 点云可以被重发和整理，但当前 3D 避障默认直接吃 `/cloud_registered`

## 10. 第九站：地图与重定位

### 文件

`src/map_manager/src/map_saver.cpp`
`src/map_manager/src/map_loader.cpp`
`src/relocalizer/src/icp_relocalizer.cpp`

### 为什么放后面

因为这些对系统很重要，但不是你第一轮读码最容易卡住理解的位置。

你第一轮应该先把“飞起来”和“避开障碍”的主链看通。

## 11. 你每读完一个文件都要记一张卡片

建议你自己做一个简单表格，每个文件只记 4 件事：

1. 输入
2. 输出
3. 核心函数
4. 它在整个链路里的位置

例如：

### `local_3d_avoidance.cpp`

- 输入：`/cmd_vel`、`/cloud_registered`、`/odom_filtered`
- 输出：`/cmd_vel_safe`
- 核心函数：`analyze_obstacles()`、`apply_avoidance()`
- 位置：任务层和飞控桥之间

## 12. 最后给你的实际建议

第一轮读码只追主链，不追全量细节。

最值得先读通的 5 个文件是：

1. `launch/auto_flight/bringup_all.launch.py`
2. `src/px4_offboard_bridge/launch/offboard_bridge.launch.py`
3. `src/px4_offboard_bridge/src/local_3d_avoidance.cpp`
4. `src/waypoint_planner/src/mission_executor.cpp`
5. `src/px4_offboard_bridge/src/cmdvel_to_setpoint.cpp`

这 5 个文件通了，你对这个项目的主控制链就已经基本有数了。
