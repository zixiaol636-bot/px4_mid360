#include "warehouse_utils/yaml_utils.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <stdexcept>
#include <string>

//看快快快
// ¿ªÆô
namespace warehouse_utils
{
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
