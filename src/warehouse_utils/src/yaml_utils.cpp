#include "warehouse_utils/yaml_utils.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <stdexcept>

namespace warehouse_utils
{

namespace
{

void validate_sequence_size(const YAML::Node & node, std::size_t expected, const char * field_name)
{
  if (!node || !node.IsSequence() || node.size() != expected) {
    throw std::runtime_error(std::string("Invalid waypoint field: ") + field_name);
  }
}

}  // namespace

bool save_waypoints(const std::string & filepath, const std::vector<Waypoint> & waypoints)
{
  try {
    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (const auto & wp : waypoints) {
      out << YAML::BeginMap;
      out << YAML::Key << "frame_id" << YAML::Value << wp.pose.header.frame_id;
      out << YAML::Key << "position" << YAML::Value
          << YAML::Flow << YAML::BeginSeq
          << wp.pose.pose.position.x
          << wp.pose.pose.position.y
          << wp.pose.pose.position.z
          << YAML::EndSeq;
      out << YAML::Key << "orientation" << YAML::Value
          << YAML::Flow << YAML::BeginSeq
          << wp.pose.pose.orientation.x
          << wp.pose.pose.orientation.y
          << wp.pose.pose.orientation.z
          << wp.pose.pose.orientation.w
          << YAML::EndSeq;
      out << YAML::Key << "hover_time" << YAML::Value << wp.hover_time;
      out << YAML::Key << "acceptance_radius" << YAML::Value << wp.acceptance_radius;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;

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
    const YAML::Node config = YAML::LoadFile(filepath);
    if (!config.IsSequence()) {
      throw std::runtime_error("Waypoint file must contain a sequence");
    }

    for (const auto & node : config) {
      validate_sequence_size(node["position"], 3, "position");
      validate_sequence_size(node["orientation"], 4, "orientation");

      Waypoint wp;
      wp.pose.header.frame_id = node["frame_id"].as<std::string>("map");
      wp.pose.pose.position.x = node["position"][0].as<double>();
      wp.pose.pose.position.y = node["position"][1].as<double>();
      wp.pose.pose.position.z = node["position"][2].as<double>();
      wp.pose.pose.orientation.x = node["orientation"][0].as<double>();
      wp.pose.pose.orientation.y = node["orientation"][1].as<double>();
      wp.pose.pose.orientation.z = node["orientation"][2].as<double>();
      wp.pose.pose.orientation.w = node["orientation"][3].as<double>();
      wp.hover_time = node["hover_time"].as<double>(0.0);
      wp.acceptance_radius = node["acceptance_radius"].as<double>(1.0);
      waypoints.push_back(wp);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("yaml_utils"), "Failed to load waypoints: %s", e.what());
  }
  return waypoints;
}

double load_param_double(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  double default_val)
{
  double val = default_val;
  node->get_parameter_or(param_name, val, default_val);
  return val;
}

std::string load_param_string(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  const std::string & default_val)
{
  std::string val = default_val;
  node->get_parameter_or(param_name, val, default_val);
  return val;
}

}  // namespace warehouse_utils
