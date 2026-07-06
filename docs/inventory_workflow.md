# 六仓室内盘库工作流

这份文档专门对应你的真实业务目标:

- PX4 + MID360 + 机载电脑
- 六个连续并排仓库
- 飞手先手动起飞和进出仓
- 仓内按类似“几”字形轨迹完成盘库
- 最后一个仓库出来后自动回到起飞点
- 支持离线建图，也支持可选的在线建图模式
- 主流程采用“人工/生成航点 + Offboard 自动飞行 + 3D 局部避障”
- 同时保留 A* + DWB 自动规划作为技术储备，可用于其他自主飞行场景

## 1. 现在项目能做到什么

按代码结构和当前补齐后的能力看，这个仓库现在已经具备下面几条主链:

### 主业务链: 六仓盘库

1. 手飞录包
2. 基于 FAST-LIO2 回放建图
3. 保存 PCD 和 Nav2 栅格图
4. 根据仓库布局文件生成六仓盘库航点
5. 启动定位、PX4 桥接、避障、安全监控
6. Offboard 模式下执行航点任务
7. 任务结束后自动 RTL 回到起飞点

### 可选链: 在线建图

1. 直接启动 MID360 + FAST-LIO2 在线建图
2. 一边飞一边累积地图
3. 按需保存为 PCD

### 技术储备链: 自动规划

1. 基于 `map.yaml` 栅格图做 A* 搜索
2. 估算访问不同仓库入口点的总代价
3. 优化六仓访问顺序
4. Nav2 保留 DWB 局部规划能力，适合其他点到点自动飞行场景

## 2. 还不能把它说成“已经完全实仓可交付”的地方

这部分我必须说实话。当前仓库已经从“概念拼装”推进到“可工程联调的版本”，但还不能仅凭静态改代码就宣布完全实仓可交付，原因有三类:

### 2.1 现场参数还没被你的真实仓库尺寸替换

`config/missions/six_warehouse_layout.yaml` 现在是示例布局，里面的:

- 六个仓库中心点
- 仓长
- 仓宽
- 航向角
- 盘库高度
- 走廊间距

都需要改成你的真实仓库参数。

### 2.2 我这边没法完成真实编译和飞行验证

当前环境里缺少可用的 `cmake` / `colcon`，而且 `python` 运行环境也不可直接用于常规 ROS 验证，所以我这次做的是“代码补齐 + 静态一致性检查”，不是“已在本机完整编译通过并跑飞”。

### 2.3 主盘库链和 Nav2 技术储备链不是同一条执行链

这点很重要:

- 你的主盘库链现在是:
  `生成/人工航点 -> mission_executor -> /cmd_vel -> local_3d_avoidance -> PX4`
- 你的技术储备自动规划链是:
  `Nav2 A* -> DWB -> 点到点自动规划`

也就是说，盘库主流程不是让 Nav2 逐个替你“自动想怎么扫仓”，而是按你定义好的盘库策略飞，局部再由避障层兜底。这其实更符合盘库业务，因为仓内扫描轨迹通常是工艺要求，不适合完全交给自由规划器。

## 3. 六仓布局文件怎么用

布局文件:

`config/missions/six_warehouse_layout.yaml`

它描述的是“六个仓库长什么样、排成什么样、从哪里开始盘”。

关键字段:

- `start_pose`: 起飞点/任务起始点参考
- `warehouses[].center`: 每个仓库的中心
- `warehouses[].length`: 仓深
- `warehouses[].width`: 仓宽
- `warehouses[].yaw_deg`: 仓库朝向
- `warehouses[].altitude`: 盘库飞行高度
- `warehouses[].lane_spacing`: “几”字形扫线间距
- `warehouses[].margin`: 离墙余量

如果你的六个仓是连续并排的，这个文件就是主入口之一。

## 4. 怎么生成六仓盘库任务

### 4.1 直接按布局生成盘库航点

```bash
ros2 run waypoint_planner generate_inventory_mission.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./config/waypoints/six_warehouse_inventory.yaml
```

这个脚本会做两件事:

1. 按仓库顺序依次生成每个仓内的“几”字形扫描轨迹
2. 输出给 `mission_executor` 可直接执行的 waypoint YAML

说明:

- 任务执行结束后，不需要你在航点里手工再加一个回家点
- `mission_executor` 在最后一个航点完成后会自动进入 RTL
- RTL 的 home pose 是任务启动瞬间记录的当前位置

### 4.2 先优化访问顺序，再生成任务

如果你想把“六仓访问顺序最短”作为技术储备或其他场景优化，可以先运行:

```bash
ros2 run waypoint_planner optimize_inventory_route.py \
  ./config/missions/six_warehouse_layout.yaml \
  ./maps/map.yaml \
  --output-layout ./config/missions/six_warehouse_layout_optimized.yaml
```

然后再生成:

```bash
ros2 run waypoint_planner generate_inventory_mission.py \
  ./config/missions/six_warehouse_layout_optimized.yaml \
  ./config/waypoints/six_warehouse_inventory.yaml
```

这个优化脚本当前做的是:

- 从 `map.yaml + map.pgm` 读取占据栅格
- 取每个仓库入口锚点
- 用 A* 估算从起点到各仓、仓与仓之间的代价
- 给出六个仓库的推荐访问顺序

这条链更偏“调度顺序优化”，不是替代仓内扫描轨迹本身。

## 5. 离线建图工作流

### 第一步: 手飞录包

你先手动飞完整个场地，至少把:

- 起飞区
- 六个仓的入口
- 仓内主要通道
- 仓间过渡区域

都扫到。

### 第二步: 回放建图

```bash
ros2 launch px4_mid360 build_map_offline.launch.py \
  bag_path:=./bags/warehouse_mapping \
  map_path:=./maps/warehouse_map.pcd
```

### 第三步: 保存并转 Nav2 栅格图

```bash
ros2 launch px4_mid360 save_map.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  output_dir:=./maps
```

产物通常包括:

- `warehouse_map.pcd`
- `map.pgm`
- `map.yaml`

## 6. 在线建图工作流

如果你想做“飞行时同步建图”的可选模式，现在仓库里也已经补了入口:

```bash
ros2 launch px4_mid360 online_mapping.launch.py \
  map_path:=./maps/warehouse_online_map.pcd
```

这条链适合:

- 前期快速采图
- 新场地试扫
- 做在线建图模式储备

但对正式盘库，我仍然建议优先使用“先建好图，再执行自动盘库”的离线路径，因为风险更可控。

## 7. 自动盘库执行链

先启动全系统:

```bash
ros2 launch px4_mid360 bringup_all.launch.py \
  map_path:=./maps/warehouse_map.pcd \
  map_grid_path:=./maps/map.yaml
```

再启动任务:

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/six_warehouse_inventory.yaml
```

如果你的现场流程是“飞手先手动起飞，再切自动任务”，建议这样启动：

```bash
ros2 launch px4_mid360 offboard_mission.launch.py \
  waypoint_file:=./config/waypoints/six_warehouse_inventory.yaml \
  takeoff_altitude:=0.0
```

这样 `mission_executor` 不会再额外上拉一段起飞高度，而是从当前任务接管点直接开始飞任务。

当前主控制链是:

```text
mission_executor
  -> /cmd_vel
  -> local_3d_avoidance
  -> /cmd_vel_safe
  -> cmdvel_to_setpoint
  -> /mavros/setpoint_velocity/cmd_vel
  -> PX4
```

它的含义是:

- 任务层负责“该往哪飞”
- 3D 避障层负责“前面有东西时怎么临时绕一下/减速/抬高/侧移”
- PX4 桥负责把 ROS 速度命令转成飞控能用的控制指令

## 8. DWB 局部避障和 3D 局部避障的关系

你要的是“人工设置航点，并能局部避障”，这个目标现在拆成了两层:

### 盘库主流程里真正生效的

- `local_3d_avoidance`

它直接看点云，对 `/cmd_vel` 做 3D 安全修正，更适合无人机。

### 程序里保留的 Nav2 技术储备

- `planner_server.GridBased.use_astar: true`
- `controller_server.FollowPath.plugin: dwb_core::DWBLocalPlanner`

也就是说:

- A* 在 `config/nav2/nav2_params.yaml` 里已经打开
- DWB 也已经配置好
- 这套更适合“地图上从 A 点去 B 点”的自主导航储备

但对你的盘库业务，我建议继续把主链放在“预定义盘库轨迹 + 3D 避障”上，因为更稳、更符合实际盘库路线约束。

## 9. 我对当前项目能力的结论

### 现在已经可以说“基本具备”

- 离线建图
- 可选在线建图
- 六仓布局描述
- 六仓“几”字盘库航点自动生成
- 任务执行后自动返航
- 3D 局部避障
- Nav2 A* + DWB 技术储备

### 现在还不能直接说“零风险落地”

- 没完成你的真实硬件实测
- 没用真实仓库尺寸替换示例配置
- 没做现场参数整定闭环
- 没在当前环境里完成实际编译/跑通验证

## 10. 落地前最后建议

你后面按这个顺序推进最稳:

1. 先把 `six_warehouse_layout.yaml` 改成真实仓库尺寸
2. 先离线建图并固定地图
3. 生成六仓盘库任务
4. 做不挂桨/低风险状态的话题联调
5. 做低速、低高度、单仓测试
6. 再扩展到六仓连续自动盘库
7. 最后再把在线建图和自动规划作为增强功能逐步上线
