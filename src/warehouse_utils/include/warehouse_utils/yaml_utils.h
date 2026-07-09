#ifndef WAREHOUSE_UTILS__YAML_UTILS_H_
#define WAREHOUSE_UTILS__YAML_UTILS_H_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace warehouse_utils
{

struct Waypoint
{
  geometry_msgs::msg::PoseStamped pose;
  double hover_time{0.0};
  double acceptance_radius{1.0};
};

bool save_waypoints(const std::string & filepath, const std::vector<Waypoint> & waypoints);

std::vector<Waypoint> load_waypoints(const std::string & filepath);

double load_param_double(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  double default_val = 0.0);

std::string load_param_string(
  const rclcpp::Node::SharedPtr & node,
  const std::string & param_name,
  const std::string & default_val = "");

}  // namespace warehouse_utils

#endif  // WAREHOUSE_UTILS__YAML_UTILS_H_
