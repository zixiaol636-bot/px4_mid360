# 学习者全流程手册

这份文档是给“要把这个项目学明白、跑起来、以后还能自己改”的人写的。

目标不是只告诉你命令怎么敲，而是帮你建立三件事：

1. 这个项目到底在解决什么问题。
2. 代码应该按什么顺序看，才不会一上来就陷进细节。
3. 作为使用者，怎么把完整流程真正跑通。

## 1. 先别急着看代码，先知道系统在干什么

这个项目本质上是在做一套室内无人机系统，核心链路可以先压缩成两条：

### 感知与定位链

```text
MID360 -> FAST-LIO2 -> /Odometry
       -> fast_lio_bridge -> /odom_filtered
       -> vision_pose_bridge -> /mavros/vision_pose/pose
       -> PX4 EKF
```

### 控制链

```text
上层控制(任务节点 / Nav2 / 调试节点)
    -> /cmd_vel
    -> local_3d_avoidance
    -> /cmd_vel_safe
    -> cmdvel_to_setpoint
    -> /mavros/setpoint_velocity/cmd_vel
    -> PX4 Offboard
```

你学习这个项目时，只要先盯住这两条链，很多文件就不会显得散。

## 2. 你应该先学什么

如果你一上来就从 `src/` 里随机翻 C++ 文件，很容易越看越乱。

推荐顺序是：

1. 先看根目录 `README.md`
2. 再看顶层启动文件 `launch/auto_flight/bringup_all.launch.py`
3. 再看 `src/px4_offboard_bridge/config/offboard_params.yaml`
4. 再看 `src/waypoint_planner/src/mission_executor.cpp`
5. 再看 `src/fast_lio_bridge/src/odometry_relay.cpp`
6. 再看 `src/map_manager/src/map_saver.cpp`
7. 最后看 `src/relocalizer/src/icp_relocalizer.cpp`

这样看的原因很简单：

- `README.md` 负责告诉你系统怎么用
- launch 文件负责告诉你系统怎么拼起来
- config 文件负责告诉你运行时怎么调
- 节点源码负责告诉你每个环节具体做了什么

## 3. 先建立“包”的脑图

你可以把这个仓库粗略分成 6 层：

### 第 1 层：公共工具层

`src/warehouse_utils`

作用：

- YAML 读写
- 坐标/姿态工具
- 给其他包复用

学习重点：

- 不要把它当业务逻辑
- 把它看成“底层胶水”

### 第 2 层：传感器与里程计桥接层

`src/mid360_driver_wrapper`
`src/fast_lio_bridge`

作用：

- 拉起 MID360 驱动
- 接 FAST-LIO2 输出
- 把原始输出整理成下游更稳定的格式

学习重点：

- 看清 `/Odometry` 是怎么变成 `/odom_filtered` 的
- 看清点云是怎么转发的

### 第 3 层：地图层

`src/map_manager`

作用：

- 保存 `.pcd`
- 加载 `.pcd`
- 转成 Nav2 使用的 `map.pgm + map.yaml`

学习重点：

- 这个包不负责“飞”
- 它负责给定位和导航提供地图资产

### 第 4 层：重定位层

`src/relocalizer`

作用：

- 把当前扫描和已有地图对齐
- 发布 `map -> odom` 的 TF

学习重点：

- 理解为什么有了 FAST-LIO2 还要重定位
- 理解 `map` 和 `odom` 不是一个概念

### 第 5 层：飞控桥接层

`src/px4_offboard_bridge`

作用：

- 把定位送给 PX4
- 把 `/cmd_vel` 先变成经过 3D 局部避障的 `/cmd_vel_safe`
- 再把 `/cmd_vel_safe` 转成 MAVROS/PX4 能吃的速度控制

学习重点：

- `vision_pose_bridge.cpp`
- `local_3d_avoidance.cpp`
- `cmdvel_to_setpoint.cpp`

这是你最值得先看懂的一层，因为它是“软件世界”和“飞控世界”的交界处。

### 第 6 层：任务与安全层

`src/waypoint_planner`
`src/safety_monitor`

作用：

- 保存航点
- 跑任务状态机
- 做电池、通信、EKF、围栏的兜底监控

学习重点：

- `mission_executor.cpp` 是任务大脑
- `safety_monitor` 是运行保护，不是完整飞控替代品

## 4. 代码应该怎么读

推荐你用“从外到内”的方法。

### 第一步：先看 launch

先读：

- `launch/auto_flight/bringup_all.launch.py`
- `launch/auto_flight/localize_only.launch.py`
- `launch/auto_flight/offboard_mission.launch.py`

你要回答三个问题：

1. 系统启动时一共拉起了哪些节点？
2. 哪些节点是基础设施，哪些节点是任务逻辑？
3. 哪些参数是从 launch 传给节点的？

### 第二步：再看 topic 和 service 名字

你读代码时，不要先纠结算法细节，先记住这些关键接口：

- `/Odometry`
- `/odom_filtered`
- `/map_cloud`
- `/mavros/vision_pose/pose`
- `/cmd_vel`
- `/cmd_vel_safe`
- `/mavros/setpoint_velocity/cmd_vel`
- `/mission/start`
- `/mission/pause`
- `/mission/abort`
- `/waypoint/save`
- `/waypoint/load`

你能把这些接口在脑子里连起来，系统就已经清楚一半了。

### 第三步：最后再进节点源码

建议顺序：

1. `vision_pose_bridge.cpp`
2. `cmdvel_to_setpoint.cpp`
3. `mission_executor.cpp`
4. `waypoint_manager.cpp`
5. `odometry_relay.cpp`
6. `map_saver.cpp`
7. `icp_relocalizer.cpp`

每看一个文件，都只问四件事：

1. 它订阅什么？
2. 它发布什么？
3. 它提供什么服务？
4. 它保存了什么状态？

这样比一行一行啃代码更容易建立系统感。

## 5. 作为学习者，建议分 4 个阶段来跑

不要一上来就“全系统 + 真机 + 自动任务”。

### 阶段 A：先只学目录和启动链

目标：

- 知道每个包负责什么
- 知道 `bringup_all.launch.py` 会启动什么

你可以做的事：

- 通读 `README.md`
- 通读这份文档
- 打开顶层 launch 文件，把每个 Include 的包记下来

### 阶段 B：先跑定位，不跑任务

目标：

- 确认地图、点云、里程计、重定位链是通的

命令：

```bash
ros2 launch px4_mid360 localize_only.launch.py \
  map_path:=./maps/warehouse_map.pcd
```

你应该重点观察：

- `/odom_filtered`
- `/map_cloud`
- `/tf`
- `/mavros/vision_pose/pose`

如果这一阶段都不稳定，就不要急着跑任务。

### 阶段 C：跑全系统，但先不飞任务

目标：

- 确认 Nav2、桥接层、安全层全部正常启动

命令：

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  map_grid_path:=./maps/map.yaml
```

你应该重点观察：

- `/mavros/state`
- `/odom_filtered`
- `/mavros/vision_pose/pose`
- `/cmd_vel`
- `/cmd_vel_safe`
- `/mavros/setpoint_velocity/cmd_vel`

这个阶段最重要的是确认：

- 定位在更新
- 飞控收到视觉位姿
- 控制链没有断
- 没有明显节点互相抢控制

### 阶段 D：最后再跑航点任务

命令：

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/warehouse_example.yaml
```

现在的任务节点会自动调用 `/mission/start`，不用你再手动 call 一次。

你应该重点观察：

- `/mission/state`
- `/cmd_vel`
- `/mavros/setpoint_velocity/cmd_vel`

## 6. 完整使用流程怎么走

下面给你一条更像真实项目操作的全流程。

### 第 1 步：准备环境

```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

如果是第一次配置，还要参考：

- `docs/setup_guide.md`

### 第 2 步：准备地图

如果你已经有地图文件：

- `warehouse_map.pcd`
- `map.yaml`
- `map.pgm`

可以直接跳到定位和任务阶段。

如果你没有地图，先离线建图。

#### 2.1 录包

```bash
ros2 launch px4_mid360 record_bag.launch.py
```

#### 2.2 回放建图

```bash
ros2 launch px4_mid360 build_map_offline.launch.py \
  bag_path:=./bags/warehouse_mapping \
  map_path:=./maps/warehouse_map.pcd
```

#### 2.3 保存并转成 Nav2 地图

```bash
ros2 launch px4_mid360 save_map.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  output_dir:=./maps
```

最终你应该得到：

- `maps/warehouse_map.pcd`
- `maps/map.yaml`
- `maps/map.pgm`

### 第 3 步：准备航点

如果你已经有航点 YAML，可以直接用。

如果没有，建议用 RViz 点选方式生成：

```bash
ros2 run waypoint_planner interactive_waypoints.py --ros-args \
  -p waypoints_file:=./config/waypoints/warehouse_example.yaml
```

操作方式：

1. 打开 RViz
2. 使用 `Publish Point`
3. 往 `/clicked_point` 点位置
4. 退出脚本时自动保存

### 第 4 步：启动定位或全系统

如果你只是验证定位：

```bash
ros2 launch px4_mid360 localize_only.launch.py \
  map_path:=./maps/warehouse_map.pcd
```

如果你要准备执行任务：

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  map_grid_path:=./maps/map.yaml
```

### 第 5 步：启动任务

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/warehouse_example.yaml
```

### 第 6 步：任务中控制

你可以用 service 控制任务状态：

```bash
ros2 service call /mission/pause std_srvs/srv/Trigger "{}"
ros2 service call /mission/abort std_srvs/srv/Trigger "{}"
```

说明：

- 再次调用 `/mission/pause` 会从暂停恢复
- `/mission/abort` 会进入 RTL

## 7. 学这个项目时，你最该盯的几个文件

### `launch/auto_flight/bringup_all.launch.py`

学什么：

- 系统装配方式
- 依赖顺序
- 关键参数入口

### `src/waypoint_planner/src/mission_executor.cpp`

学什么：

- 状态机怎么写
- 航点任务怎么组织
- 暂停、恢复、RTL 怎么实现

### `src/px4_offboard_bridge/src/local_3d_avoidance.cpp`

学什么：

- 3D 局部避障是如何插进控制链的
- 点云是如何变成避障修正量的
- 为什么这里是“人工航点 + 3D 避障”的关键节点

### `src/px4_offboard_bridge/src/cmdvel_to_setpoint.cpp`

学什么：

- 避障后的安全速度怎么被限幅和平滑
- 为什么项目统一对外暴露 `/cmd_vel`

### `src/fast_lio_bridge/src/odometry_relay.cpp`

学什么：

- 传感器定位输出为什么还要再包一层
- 下游为什么更喜欢统一后的 `/odom_filtered`

### `src/relocalizer/src/icp_relocalizer.cpp`

学什么：

- 地图坐标系如何和当前里程计坐标系对齐

## 8. 调试时最常见的观察点

如果“项目启动了，但不工作”，优先看这几个点：

### 定位问题

看：

- `/Odometry`
- `/odom_filtered`
- `/tf`
- `/map_cloud`

如果这些都不正常，先别看任务层。

### 飞控桥接问题

看：

- `/mavros/state`
- `/mavros/vision_pose/pose`
- `/cmd_vel_safe`
- `/mavros/setpoint_velocity/cmd_vel`

如果视觉位姿没送到 PX4，任务层写得再好也飞不起来。

### 任务问题

看：

- `/mission/state`
- `/cmd_vel`
- `/cmd_vel_safe`

如果状态在变但 `/cmd_vel` 没输出，问题多半在任务节点内部。

如果 `/cmd_vel` 有输出但 `/cmd_vel_safe` 没变化或异常，问题多半在 3D 避障层。

如果 `/cmd_vel_safe` 有输出但飞控不响应，问题多半在桥接层或 PX4。

## 9. 学完第一轮之后，你下一步应该怎么进阶

当你已经能把项目跑起来，下一步可以按这个顺序继续：

1. 给 `mission_executor` 补单元测试
2. 给 launch 补冒烟测试
3. 把任务层逐步从“直接发速度”升级成“发 Nav2 goal”
4. 给安全层补一个真正的 command arbiter
5. 再做 SITL 或实机联调

## 10. 对学习者最重要的一句建议

不要试图一次性学完整个仓库。

更好的方法是：

1. 先认清两条主链：定位链、控制链
2. 再认清 6 层结构：工具、桥接、地图、重定位、飞控桥、任务安全
3. 再按“启动文件 -> 接口 -> 节点源码”的顺序深入
4. 每学完一层，就立刻用一个最小命令验证它

如果你愿意，我下一步还可以继续给你补一份“带文件路径的读码路线图”，把每个阶段该看的函数、参数、话题名再拆得更细。
