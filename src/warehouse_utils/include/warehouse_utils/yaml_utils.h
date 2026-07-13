#ifndef WAREHOUSE_UTILS__YAML_UTILS_H_
#define WAREHOUSE_UTILS__YAML_UTILS_H_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

// 将航点 YAML 工具归入 warehouse_utils 模块，避免与其他包的同名类型或函数冲突。
namespace warehouse_utils
{

// 任务执行器使用的最小航点数据结构，可与 YAML 航点文件相互转换。
struct Waypoint
{
  // 目标位置和目标姿态；frame_id 通常为 FAST-LIO 发布的 map 坐标系。
  geometry_msgs::msg::PoseStamped pose;
  // 到达航点后需要悬停的时长，单位为秒；0.0 表示不额外悬停。
  double hover_time{0.0};
  // 判定“已到达航点”的位置容差半径，单位为米。
  double acceptance_radius{1.0};
};

// 将航点列表保存为 YAML 文件。写入成功返回 true，失败时记录 ROS 错误日志并返回 false。
bool save_waypoints(const std::string & filepath, const std::vector<Waypoint> & waypoints);

// 从 YAML 文件读取航点列表。格式错误或读取失败时记录 ROS 错误日志并返回空列表。
std::vector<Waypoint> load_waypoints(const std::string & filepath);

// 读取 double 类型 ROS 参数；参数不存在时返回 default_val。
double load_param_double(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  double default_val = 0.0);

// 读取 string 类型 ROS 参数；参数不存在时返回 default_val。
std::string load_param_string(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  const std::string & default_val = "");

}  // namespace warehouse_utils

#endif  // WAREHOUSE_UTILS__YAML_UTILS_H_
