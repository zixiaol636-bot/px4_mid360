# 实操手册

这份文档按“实验指导书”的方式来写。目标不是只给你命令，而是让你每一步都知道：

1. 为什么要做这一步
2. 应该执行什么命令
3. 正常现象是什么
4. 不正常时先看哪里

## 实验 0：先理解这次要验证什么

当前主控制链是：

```text
航点任务 / 人工速度
    -> /cmd_vel
    -> local_3d_avoidance
    -> /cmd_vel_safe
    -> cmdvel_to_setpoint
    -> /mavros/setpoint_velocity/cmd_vel
    -> PX4
```

你要分清 3 层：

- `/cmd_vel`：上层期望速度
- `/cmd_vel_safe`：经过 3D 局部避障后的安全速度
- `/mavros/setpoint_velocity/cmd_vel`：最终发给飞控的速度

## 实验 1：编译环境准备

### 目标

确认工作区能编译，而且新增节点已经被编进去。

### 命令

```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 预期现象

- `px4_offboard_bridge` 编译通过
- `waypoint_planner` 编译通过
- 可以找到新增脚本和节点

### 追加检查

```bash
ros2 pkg executables px4_offboard_bridge
ros2 pkg executables waypoint_planner
```

### 你应该能看到

- `vision_pose_bridge`
- `local_3d_avoidance`
- `cmdvel_to_setpoint`
- `mission_executor`
- `waypoint_manager`

### 如果失败

优先检查：

- `src/px4_offboard_bridge/CMakeLists.txt`
- `src/waypoint_planner/CMakeLists.txt`

## 实验 1.5：生成六仓盘库任务

### 目标

把“六个连续并排仓库”的业务描述先变成可执行任务文件。

### 第一步：检查布局文件

先看：

`config/missions/six_warehouse_layout.yaml`

重点确认这些字段是不是你的真实场地：

- `center`
- `length`
- `width`
- `yaw_deg`
- `altitude`
- `lane_spacing`
- `margin`

### 第二步：直接生成任务

```bash
ros2 run waypoint_planner generate_inventory_mission.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./config/waypoints/six_warehouse_inventory.yaml
```

### 预期现象

- 终端打印 `Generated N waypoints`
- 生成文件 `config/waypoints/six_warehouse_inventory.yaml`

### 第三步：人工抽查结果

重点看 3 件事：

1. 是否按照仓库顺序生成
2. 每个仓内部是否呈往返“几”字形
3. 高度是否符合你的盘库设计

### 第四步：可选顺序优化

```bash
ros2 run waypoint_planner optimize_inventory_route.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./maps/map.yaml \
  --output-layout ./config/missions/six_warehouse_layout_optimized.yaml
```

再生成：

```bash
ros2 run waypoint_planner generate_inventory_mission.py \
  ./config/missions/six_warehouse_layout_optimized.yaml \
  ./config/waypoints/six_warehouse_inventory.yaml
```

### 预期现象

- 终端会打印推荐访问顺序
- 会输出优化后的 layout 文件

### 你要理解的关键点

- 这里优化的是“先去哪个仓更合理”
- 不是让规划器自由决定仓内扫线
- 仓内“几”字轨迹仍然是按盘库工艺生成的

## 实验 2：只验证定位链

### 目标

先确认定位和点云正常，再谈避障和任务。

### 命令

```bash
ros2 launch px4_mid360 localize_only.launch.py \
  map_path:=./maps/warehouse_map.pcd
```

### 追加观察

```bash
ros2 topic echo /odom_filtered --once
ros2 topic echo /mavros/vision_pose/pose --once
```

### 预期现象

- `/odom_filtered` 持续更新
- `/mavros/vision_pose/pose` 有输出

### 如果失败

优先看：

- MID360 驱动是否起来
- FAST-LIO2 是否正常输出 `/Odometry`
- `vision_pose_bridge` 是否启动

## 实验 3：验证 3D 避障层是否真的插进链路

### 目标

确认系统不再是“上层速度直接喂给飞控”。

### 命令

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  map_grid_path:=./maps/map.yaml
```

### 追加观察

```bash
ros2 topic list | grep cmd_vel
```

### 你应该能看到

- `/cmd_vel`
- `/cmd_vel_safe`
- `/mavros/setpoint_velocity/cmd_vel`

### 再分别看一次

```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /cmd_vel_safe --once
ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once
```

### 预期现象

- `/cmd_vel` 是上游原始期望速度
- `/cmd_vel_safe` 是避障后速度
- `/mavros/setpoint_velocity/cmd_vel` 是最终平滑后的速度

### 如果没有 `/cmd_vel_safe`

优先看：

- `src/px4_offboard_bridge/launch/offboard_bridge.launch.py`
- `src/px4_offboard_bridge/config/offboard_params.yaml`

## 实验 4：手工验证 3D 避障是否工作

### 目标

不依赖任务层，直接手发速度，看局部避障是否能修改输出。

### 前提

系统已经运行在 `bringup_all.launch.py`

### 操作

先在无人机前方放一个明显障碍物，再发一个持续前进命令。

### 命令

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.8, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10
```

另开一个终端观察：

```bash
ros2 topic echo /cmd_vel_safe
```

### 预期现象

无遮挡时：

- `/cmd_vel_safe` 大体接近 `/cmd_vel`

前方有障碍时：

- `/cmd_vel_safe.linear.x` 会被削弱
- 可能出现 `linear.y` 侧向偏移
- 可能出现 `linear.z` 上下绕障偏移

### 如果完全没变化

优先排查：

1. `/cloud_registered` 有没有数据
2. 点云 frame 和里程计是否对应
3. `local_3d_avoidance` 的 `cloud_in_body_frame` 是否配错
4. `forward_influence_distance` 是否太小

## 实验 5：调参数看避障行为变化

### 目标

学会判断问题到底是“没有避障”还是“避障太猛”。

### 重点参数

在 `src/px4_offboard_bridge/config/offboard_params.yaml` 里调：

- `forward_influence_distance`
- `side_influence_distance`
- `vertical_influence_distance`
- `safety_distance_xy`
- `repulsion_gain_xy`
- `repulsion_gain_z`
- `lateral_escape_gain`
- `vertical_escape_gain`

### 推荐调法

情况 A：太晚才开始绕

- 增大 `forward_influence_distance`
- 增大 `safety_distance_xy`

情况 B：看到障碍但侧绕不明显

- 增大 `repulsion_gain_xy`
- 增大 `lateral_escape_gain`

情况 C：需要更愿意上下绕

- 增大 `repulsion_gain_z`
- 增大 `vertical_escape_gain`

### 每次改完后

重新启动：

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  map_grid_path:=./maps/map.yaml
```

## 实验 6：验证人工航点任务链

### 目标

验证“人工设置航点 + 任务执行 + 3D 局部避障”整条链。

### 第一步：准备航点

```bash
ros2 run waypoint_planner interactive_waypoints.py --ros-args \
  -p waypoints_file:=./config/waypoints/warehouse_example.yaml
```

### 预期现象

- 在 RViz 点击后出现 waypoint marker
- 退出脚本后 YAML 被保存

### 第二步：启动全系统

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  map_grid_path:=./maps/map.yaml
```

### 第三步：启动任务

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/warehouse_example.yaml
```

### 你要同时观察

```bash
ros2 topic echo /mission/state
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_safe
ros2 topic echo /mavros/setpoint_velocity/cmd_vel
```

### 预期现象

- `/mission/state` 会切换状态
- `/cmd_vel` 会给出朝航点的期望速度
- `/cmd_vel_safe` 会在障碍存在时发生修正
- `/mavros/setpoint_velocity/cmd_vel` 会继续输出平滑后的最终速度

## 实验 7：验证六仓自动盘库链

### 目标

验证“六仓任务生成 + Offboard 执行 + 最终自动返航”。

### 第一步：生成六仓任务

```bash
ros2 run waypoint_planner generate_inventory_mission.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./config/waypoints/six_warehouse_inventory.yaml
```

### 第二步：启动全系统

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  map_grid_path:=./maps/map.yaml
```

### 第三步：执行任务

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/six_warehouse_inventory.yaml
```

如果你是先人工起飞再切自动，改成：

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/six_warehouse_inventory.yaml \
  takeoff_altitude:=0.0
```

### 你要重点观察

```bash
ros2 topic echo /mission/state
```

### 预期现象

- 先进入 `TAKEOFF`
- 再切到 `NAVIGATE`
- 每到航点后切到 `HOVER_AT_WP`
- 最后一个航点完成后切到 `RTL`
- 回到起点附近后回到 `IDLE`

### 如果最后没有返航

优先排查：

1. 任务开始时是否已经有有效 `/odom_filtered`
2. `mission_executor` 是否成功记录 home pose
3. 是否在任务中途触发了 timeout 或 pause 异常

## 实验 8：验证暂停和中止

### 目标

确认任务控制服务在接入避障后仍然有效。

### 命令

暂停：

```bash
ros2 service call /mission/pause std_srvs/srv/Trigger "{}"
```

恢复：

```bash
ros2 service call /mission/pause std_srvs/srv/Trigger "{}"
```

中止返航：

```bash
ros2 service call /mission/abort std_srvs/srv/Trigger "{}"
```

### 预期现象

- 暂停时 `/cmd_vel` 和 `/cmd_vel_safe` 逐步收敛到零
- 恢复后继续执行
- 中止后进入 RTL 逻辑

## 实验 9：验证自动规划技术储备

### 目标

确认仓库里确实保留了 A* + DWB 这条技术储备链。

### 检查点

看 `config/nav2/nav2_params.yaml`

重点确认：

- `planner_server.GridBased.use_astar: true`
- `controller_server.FollowPath.plugin: dwb_core::DWBLocalPlanner`

### 你要理解

- 这条链主要用于点到点自动规划储备
- 不是当前六仓盘库主流程的执行核心
- 六仓盘库主流程更依赖“预定义扫描轨迹 + 3D 避障”

## 实验 10：如何判断问题出在哪一层

### 现象 1：任务在跑，但飞机不动

先看：

```bash
ros2 topic echo /cmd_vel --once
```

如果没有输出：

- 问题多半在 `mission_executor`

如果有输出，再看：

```bash
ros2 topic echo /cmd_vel_safe --once
```

如果这里没输出：

- 问题多半在 `local_3d_avoidance`

如果这里有输出，再看：

```bash
ros2 topic echo /mavros/setpoint_velocity/cmd_vel --once
```

如果这里没输出：

- 问题多半在 `cmdvel_to_setpoint`

### 现象 2：有速度，但完全不避障

先看：

```bash
ros2 topic echo /cloud_registered --once
```

如果没有点云：

- 先别看避障参数，先修感知链

如果点云有，但 `/cmd_vel_safe` 和 `/cmd_vel` 永远一样：

- 优先检查 `local_3d_avoidance` 参数

### 现象 3：避障太激进，轨迹发抖

优先减小：

- `repulsion_gain_xy`
- `repulsion_gain_z`
- `lateral_escape_gain`
- `vertical_escape_gain`

再视情况减小：

- `smoothing_alpha`

## 实验 11：首轮实机建议

第一轮不要追求“复杂场景也完美避障”。

建议顺序：

1. 先验证无障碍环境速度链
2. 再验证单个静态障碍绕行
3. 再验证单仓盘库
4. 再扩展到六仓连续任务

## 12. 这份实操手册最重要的一句话

每次只验证一层。

不要一次同时改：

- 地图
- 航点
- PX4 参数
- 避障参数
- 任务逻辑

这样你才能知道，到底是哪一层导致结果变化。
