#include "warehouse_utils/yaml_utils.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <stdexcept>
#include <string>

// 与头文件使用同一模块命名空间，实现其中声明的公开接口。
namespace warehouse_utils
{
// 无名命名空间中的符号只在当前 .cpp 文件可见，用于隐藏读取、校验等内部实现细节。
namespace
{

// 统一管理 YAML 字段名，确保读取和写出使用完全相同的文件格式。
constexpr char kFrameIdField[] = "frame_id";
constexpr char kPositionField[] = "position";
constexpr char kOrientationField[] = "orientation";
constexpr char kHoverTimeField[] = "hover_time";
constexpr char kAcceptanceRadiusField[] = "acceptance_radius";
constexpr char kDefaultFrameId[] = "map";

void validate_sequence_size(
  const YAML::Node & node,
  std::size_t expected,
  const char * field_name)
{
  // 位置和四元数分量的个数与顺序会直接影响飞行任务；在此拒绝格式错误的
  // 输入，避免不完整航点进入任务执行器。
  if (!node || !node.IsSequence() || node.size() != expected) {
    throw std::runtime_error(std::string("Invalid waypoint field: ") + field_name);
  }
}

void emit_waypoint(YAML::Emitter & out, const Waypoint & wp)
{
  // 顶层航点列表中的每个元素，都以一个 YAML 映射（键值对象）写出。
  out << YAML::BeginMap;
  out << YAML::Key << kFrameIdField << YAML::Value << wp.pose.header.frame_id;

  // 流式序列会写成紧凑形式：position: [x, y, z]。
  out << YAML::Key << kPositionField << YAML::Value
      << YAML::Flow << YAML::BeginSeq
      << wp.pose.pose.position.x
      << wp.pose.pose.position.y
      << wp.pose.pose.position.z
      << YAML::EndSeq;

  // ROS 四元数采用 x、y、z、w 顺序，而不是欧拉角的横滚、俯仰、偏航。
  out << YAML::Key << kOrientationField << YAML::Value
      << YAML::Flow << YAML::BeginSeq
      << wp.pose.pose.orientation.x
      << wp.pose.pose.orientation.y
      << wp.pose.pose.orientation.z
      << wp.pose.pose.orientation.w
      << YAML::EndSeq;

  out << YAML::Key << kHoverTimeField << YAML::Value << wp.hover_time;
  out << YAML::Key << kAcceptanceRadiusField << YAML::Value << wp.acceptance_radius;
  out << YAML::EndMap;
}

Waypoint parse_waypoint(const YAML::Node & node, std::size_t index)
{
  // 顶层 YAML 的每一项必须是描述一个航点的键值对象，不能是单个数值或列表。
  if (!node || !node.IsMap()) {
    throw std::runtime_error("Waypoint item must be a map at index " + std::to_string(index));
  }

  validate_sequence_size(node[kPositionField], 3, kPositionField);
  validate_sequence_size(node[kOrientationField], 4, kOrientationField);

  Waypoint wp;
  // 可选字段缺失时使用默认值，兼容旧工具生成的航点文件；位置和姿态为必填
  // 字段，已在上方完成格式校验，因此这里不提供默认值。
  wp.pose.header.frame_id = node[kFrameIdField].as<std::string>(kDefaultFrameId);
  wp.pose.pose.position.x = node[kPositionField][0].as<double>();
  wp.pose.pose.position.y = node[kPositionField][1].as<double>();
  wp.pose.pose.position.z = node[kPositionField][2].as<double>();
  wp.pose.pose.orientation.x = node[kOrientationField][0].as<double>();
  wp.pose.pose.orientation.y = node[kOrientationField][1].as<double>();
  wp.pose.pose.orientation.z = node[kOrientationField][2].as<double>();
  wp.pose.pose.orientation.w = node[kOrientationField][3].as<double>();
  wp.hover_time = node[kHoverTimeField].as<double>(0.0);
  wp.acceptance_radius = node[kAcceptanceRadiusField].as<double>(1.0);
  return wp;
}

template<typename T>
T load_param(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  const T & default_val)
{
  // 参数未声明或未提供时，get_parameter_or 会保留给定默认值，调用方无需
  // 重复编写默认值处理逻辑。
  T val = default_val;
  node->get_parameter_or(param_name, val, default_val);
  return val;
}

}  // namespace

bool save_waypoints(const std::string & filepath, const std::vector<Waypoint> & waypoints)
{
  try {
    // 先在内存中生成完整 YAML 文档，再打开目标文件写入。
    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (const auto & wp : waypoints) {
      emit_waypoint(out, wp);
    }
    out << YAML::EndSeq;

    // 目录由调用方负责创建；本工具只负责把航点序列化并写入文件。
    std::ofstream fout(filepath);
    if (!fout.is_open()) {
      throw std::runtime_error("Unable to open waypoint file for writing");
    }

    fout << out.c_str();
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("yaml_utils"), "Failed to save waypoints: %s", e.what());
    return false;
  }
}

std::vector<Waypoint> load_waypoints(const std::string & filepath)
{
  std::vector<Waypoint> waypoints;

  try {
    // 文件顶层必须是列表，例如："- frame_id: map"、"- ..."。
    const YAML::Node config = YAML::LoadFile(filepath);
    if (!config.IsSequence()) {
      throw std::runtime_error("Waypoint file must contain a sequence");
    }

    // YAML 已给出航点总数，预先分配容量以避免循环中重复扩容。
    waypoints.reserve(config.size());
    std::size_t index = 0;
    for (const auto & node : config) {
      waypoints.push_back(parse_waypoint(node, index));
      ++index;
    }
  } catch (const std::exception & e) {
    // 错误航点文件不能导致任务执行器崩溃；函数返回空列表，由上层提示任务
    // 无法启动。
    RCLCPP_ERROR(rclcpp::get_logger("yaml_utils"), "Failed to load waypoints: %s", e.what());
  }

  return waypoints;
}

double load_param_double(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  double default_val)
{
  return load_param<double>(node, param_name, default_val);
}

std::string load_param_string(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  const std::string & default_val)
{
  return load_param<std::string>(node, param_name, default_val);
}

}  // namespace warehouse_utils
