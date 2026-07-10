/**
 * @file yaml_utils.cpp
 * @brief YAML 航点读写 + ROS2 参数提取的实现
 *
 * 保存格式: 顶层数组，每个元素是一个航点 map
 * 读取: 顶层必须数组，position 长度=3，orientation 长度=4，其余字段可缺省
 */

#include "warehouse_utils/yaml_utils.h"

#include <yaml-cpp/yaml.h>       // YAML::Node, YAML::Emitter, YAML::LoadFile

#include <fstream>               // std::ofstream 写文件
#include <stdexcept>             // std::runtime_error
#include <string>

namespace warehouse_utils
{
<<<<<<< HEAD

// 匿名命名空间 = 文件内部可见，不暴露到头文件
namespace
{

// YAML 字段名常量 — 集中管理，保存和读取共用同一套 key
constexpr char kFrameIdField[] = "frame_id";
constexpr char kPositionField[] = "position";
constexpr char kOrientationField[] = "orientation";
constexpr char kHoverTimeField[] = "hover_time";
constexpr char kAcceptanceRadiusField[] = "acceptance_radius";
constexpr char kDefaultFrameId[] = "map";                 // 没写 frame_id 时默认用 map

// 校验 YAML 数组长度: position 必须 [x,y,z] 长度=3, orientation 必须 [x,y,z,w] 长度=4
void validate_sequence_size(const YAML::Node & node, std::size_t expected, const char * field_name)
{
  if (!node || !node.IsSequence() || node.size() != expected) {
    throw std::runtime_error(std::string("Invalid waypoint field: ") + field_name);
  }
}

// 单个航点 → YAML::Emitter
void emit_waypoint(YAML::Emitter & out, const Waypoint & wp)
{
  out << YAML::BeginMap;

  out << YAML::Key << kFrameIdField << YAML::Value << wp.pose.header.frame_id;

  // position: Flow 风格写成一行 [x, y, z]
  out << YAML::Key << kPositionField << YAML::Value
      << YAML::Flow << YAML::BeginSeq
      << wp.pose.pose.position.x
      << wp.pose.pose.position.y
      << wp.pose.pose.position.z
      << YAML::EndSeq;

  // orientation: Flow 风格 [x, y, z, w] 四元数
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

// YAML::Node → Waypoint 结构体（单个航点解析）
Waypoint parse_waypoint(const YAML::Node & node, std::size_t index)
{
  if (!node || !node.IsMap()) {
    throw std::runtime_error("Waypoint item must be a map at index " + std::to_string(index));
  }

  // 先校验数组长度，再安全按下标读
  validate_sequence_size(node[kPositionField], 3, kPositionField);
  validate_sequence_size(node[kOrientationField], 4, kOrientationField);

  Waypoint wp;

  // frame_id 可缺省 → 默认 "map"
  wp.pose.header.frame_id = node[kFrameIdField].as<std::string>(kDefaultFrameId);

  // position: 已在上面校验 length=3，直接取 [0][1][2]
  wp.pose.pose.position.x = node[kPositionField][0].as<double>();
  wp.pose.pose.position.y = node[kPositionField][1].as<double>();
  wp.pose.pose.position.z = node[kPositionField][2].as<double>();

  // orientation: 四元数 qx, qy, qz, qw
  wp.pose.pose.orientation.x = node[kOrientationField][0].as<double>();
  wp.pose.pose.orientation.y = node[kOrientationField][1].as<double>();
  wp.pose.pose.orientation.z = node[kOrientationField][2].as<double>();
  wp.pose.pose.orientation.w = node[kOrientationField][3].as<double>();

  // hover_time 和 acceptance_radius 可缺省
  wp.hover_time = node[kHoverTimeField].as<double>(0.0);
  wp.acceptance_radius = node[kAcceptanceRadiusField].as<double>(1.0);

  return wp;
}

// 模板：统一定义 double 和 string 的参数读取逻辑，避免重复代码
template<typename T>
T load_param(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  const T & default_val)
{
  T val = default_val;
  node->get_parameter_or(param_name, val, default_val);  // 参数存在则覆盖
  return val;
}

}  // namespace

// ===== 公开接口 =====

bool save_waypoints(const std::string & filepath, const std::vector<Waypoint> & waypoints)
{
  try {
    YAML::Emitter out;
    out << YAML::BeginSeq;                              // 顶层数组 [
    for (const auto & wp : waypoints) {
      emit_waypoint(out, wp);                           // 逐个写入航点
    }
    out << YAML::EndSeq;                                // ]

    std::ofstream fout(filepath);                       // 打开文件
    if (!fout.is_open()) {
      throw std::runtime_error("Unable to open waypoint file for writing");
    }

    fout << out.c_str();                                // 写入 YAML 文本
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("yaml_utils"), "保存航点失败: %s", e.what());
    return false;
  }
}

std::vector<Waypoint> load_waypoints(const std::string & filepath)
{
  std::vector<Waypoint> waypoints;

  try {
    const YAML::Node config = YAML::LoadFile(filepath);         // 读取并解析 YAML
    if (!config.IsSequence()) {
      throw std::runtime_error("Waypoint file must contain a sequence");  // 顶层必须是数组
    }

    waypoints.reserve(config.size());                           // 预分配内存
    std::size_t index = 0;
    for (const auto & node : config) {
      waypoints.push_back(parse_waypoint(node, index));         // 逐项解析
      ++index;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("yaml_utils"), "加载航点失败: %s", e.what());
=======
  namespace
  {

    constexpr char kFrameIdField[] = "frame_id";
    constexpr char kPositionField[] = "position";
    constexpr char kOrientationField[] = "orientation";
    constexpr char kHoverTimeField[] = "hover_time";
    constexpr char kAcceptanceRadiusField[] = "acceptance_radius";
    constexpr char kDefaultFrameId[] = "map";

    void validate_sequence_size(const YAML::Node &node, std::size_t expected, const char *field_name)
    {
      if (!node || !node.IsSequence() || node.size() != expected)
      {
        throw std::runtime_error(std::string("Invalid waypoint field: ") + field_name);
      }
    }

    void emit_waypoint(YAML::Emitter &out, const Waypoint &wp)
    {
      out << YAML::BeginMap;
      out << YAML::Key << kFrameIdField << YAML::Value << wp.pose.header.frame_id;
      out << YAML::Key << kPositionField << YAML::Value
          << YAML::Flow << YAML::BeginSeq
          << wp.pose.pose.position.x
          << wp.pose.pose.position.y
          << wp.pose.pose.position.z
          << YAML::EndSeq;
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

    Waypoint parse_waypoint(const YAML::Node &node, std::size_t index)
    {
      if (!node || !node.IsMap())
      {
        throw std::runtime_error("Waypoint item must be a map at index " + std::to_string(index));
      }

      validate_sequence_size(node[kPositionField], 3, kPositionField);
      validate_sequence_size(node[kOrientationField], 4, kOrientationField);

      Waypoint wp;
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

    template <typename T>
    T load_param(
        const rclcpp::Node::SharedPtr &node,
        const std::string &param_name,
        const T &default_val)
    {
      T val = default_val;
      node->get_parameter_or(param_name, val, default_val);
      return val;
    }

  } // namespace

  bool save_waypoints(const std::string &filepath, const std::vector<Waypoint> &waypoints)
  {
    try
    {
      YAML::Emitter out;
      out << YAML::BeginSeq;
      for (const auto &wp : waypoints)
      {
        emit_waypoint(out, wp);
      }
      out << YAML::EndSeq;

      std::ofstream fout(filepath);
      if (!fout.is_open())
      {
        throw std::runtime_error("Unable to open waypoint file for writing");
      }

      fout << out.c_str();
      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("yaml_utils"), "Failed to save waypoints: %s", e.what());
      return false;
    }
>>>>>>> 48655d00bd63cb289277deec7930c14aa98c4537
  }

  std::vector<Waypoint> load_waypoints(const std::string &filepath)
  {
    std::vector<Waypoint> waypoints;

    try
    {
      const YAML::Node config = YAML::LoadFile(filepath);
      if (!config.IsSequence())
      {
        throw std::runtime_error("Waypoint file must contain a sequence");
      }

      waypoints.reserve(config.size());
      std::size_t index = 0;
      for (const auto &node : config)
      {
        waypoints.push_back(parse_waypoint(node, index));
        ++index;
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("yaml_utils"), "Failed to load waypoints: %s", e.what());
    }

    return waypoints;
  }

  double load_param_double(
      const rclcpp::Node::SharedPtr &node,
      const std::string &param_name,
      double default_val)
  {
    return load_param<double>(node, param_name, default_val);
  }

  std::string load_param_string(
      const rclcpp::Node::SharedPtr &node,
      const std::string &param_name,
      const std::string &default_val)
  {
    return load_param<std::string>(node, param_name, default_val);
  }

} // namespace warehouse_utils
