/**
 * @file yaml_utils.h
 * @brief YAML 参数加载与航点 I/O 工具
 *
 * 这个头文件提供了整个项目共用的航点读写和 YAML 参数提取功能。
 * 所有需要存取航点文件的包（waypoint_planner、mission_executor 等）
 * 都通过这里的统一接口操作，避免每个包各自实现一套解析逻辑。
 *
 * 使用 yaml-cpp 库做底层解析，头文件仅暴露简单的函数接口。
 */

#ifndef WAREHOUSE_UTILS__YAML_UTILS_H_
#define WAREHOUSE_UTILS__YAML_UTILS_H_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace warehouse_utils
{

/**
 * @struct Waypoint
 * @brief 单条航点数据结构
 *
 * 每条航点包含无人机需要飞到的目标位姿（map 坐标系下）、
 * 到达后的悬停时间以及判定"已到达"的距离阈值。
 *
 * 使用示例：
 * @code
 *   Waypoint wp;
 *   wp.pose.header.frame_id = "map";
 *   wp.pose.pose.position.x = 2.0;
 *   wp.pose.pose.position.y = 5.0;
 *   wp.pose.pose.position.z = 3.0;    // 飞行高度 3m
 *   wp.pose.pose.orientation.w = 1.0;  // 无旋转（朝向默认）
 *   wp.hover_time = 5.0;              // 悬停 5 秒（扫码用）
 *   wp.acceptance_radius = 0.5;       // 距目标 0.5m 内即认为到达
 * @endcode
 */
struct Waypoint
{
  /** 目标位姿，坐标系由 header.frame_id 指定（通常为 "map"） */
  geometry_msgs::msg::PoseStamped pose;

  /**
   * 在航点的悬停时间（秒）
   *
   * 用途：给扫码设备留出采集时间。设 0 表示"到达后即刻前往下一航点"，
   * 但在最后一个航点通常建议设为 0，到达后直接进入返航流程。
   */
  double hover_time{0.0};

  /**
   * 航点到达判定半径（米）
   *
   * 当无人机当前位置与目标航点的 2D 欧氏距离 <= 此值时，认为航点已到达。
   * 注意：仅比较 X-Y 平面距离，Z 轴独立判断。
   *
   * 建议值：
   *   - 狭窄通道：0.3m（精准到达）
   *   - 开阔区域：0.5m～1.0m（允许一定偏差，路径更平滑）
   */
  double acceptance_radius{1.0};
};

/**
 * @brief 将航点列表写入 YAML 文件
 *
 * 输出格式与 load_waypoints() 完全对应，可以反复存取。
 * 如果文件已存在则会被覆盖。
 *
 * @param filepath  YAML 文件的绝对或相对路径
 * @param waypoints 要保存的航点列表（可为空，写一个空序列）
 * @return true  写入成功
 * @return false 写入失败（权限不足、路径不存在等），失败原因会通过 RCLCPP_ERROR 输出
 *
 * @note YAML 文件中的 orientation 存储为 [qx, qy, qz, qw] 四元数格式
 */
bool save_waypoints(const std::string & filepath, const std::vector<Waypoint> & waypoints);

/**
 * @brief 从 YAML 文件加载航点列表
 *
 * 读取由 save_waypoints() 或人工编写的航点 YAML 文件。
 *
 * @param filepath  YAML 文件路径
 * @return 解析出的航点列表。如果文件不存在或格式错误，返回空列表，
 *         并输出 RCLCPP_ERROR 日志
 *
 * @warning 此函数不会校验航点的合理性（如是否在地图范围内），
 *          调用方应自行检查返回值是否为空。
 *
 * YAML 文件示例格式：
 * @code{.yaml}
 * ---
 * - frame_id: "map"
 *   position: [2.0, 5.0, 3.0]            # x, y, z
 *   orientation: [0.0, 0.0, 0.0, 1.0]    # qx, qy, qz, qw
 *   hover_time: 5.0
 *   acceptance_radius: 0.5
 * - frame_id: "map"
 *   position: [8.0, 5.0, 3.0]
 *   orientation: [0.0, 0.0, 0.707, 0.707]
 *   hover_time: 3.0
 *   acceptance_radius: 0.5
 * @endcode
 */
std::vector<Waypoint> load_waypoints(const std::string & filepath);

/**
 * @brief 从 ROS2 参数服务器获取 double 型参数
 *
 * 包装了 node->get_parameter_or()，简化调用并统一默认值处理。
 * 适合在节点构造函数中配合 declare_parameter 后使用。
 *
 * @param node       节点共享指针
 * @param param_name 参数名（需已 declare）
 * @param default_val 默认值（参数未设置时返回）
 * @return 参数当前值
 *
 * 使用示例：
 * @code
 *   node->declare_parameter<double>("takeoff_height", 2.0);
 *   double h = load_param_double(node, "takeoff_height", 2.0);
 * @endcode
 */
double load_param_double(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  double default_val = 0.0);

/**
 * @brief 从 ROS2 参数服务器获取 string 型参数
 *
 * 与 load_param_double() 同理，用于字符串参数提取。
 *
 * @param node       节点共享指针
 * @param param_name 参数名
 * @param default_val 默认值
 * @return 参数当前值
 */
std::string load_param_string(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  const std::string & default_val = "");

}  // namespace warehouse_utils

#endif  // WAREHOUSE_UTILS__YAML_UTILS_H_
