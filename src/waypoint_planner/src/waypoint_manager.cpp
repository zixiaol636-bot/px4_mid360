#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <waypoint_planner/msg/waypoint.hpp>
#include <waypoint_planner/srv/load_waypoints.hpp>
#include <waypoint_planner/srv/save_waypoints.hpp>
#include <warehouse_utils/yaml_utils.h>

class WaypointManager : public rclcpp::Node
{
public:
  WaypointManager() : Node("waypoint_manager")
  {
    this->declare_parameter<std::string>("waypoints_file", "waypoints.yaml");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<double>("marker_lifetime", 0.0);
    this->declare_parameter<bool>("auto_load_on_start", true);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/waypoint_markers", rclcpp::QoS(10).transient_local());

    save_srv_ = this->create_service<waypoint_planner::srv::SaveWaypoints>(
      "/waypoint/save",
      [this](
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<waypoint_planner::srv::SaveWaypoints::Request> request,
        const std::shared_ptr<waypoint_planner::srv::SaveWaypoints::Response> response)
      {
        response->success = save_waypoints(request->waypoints.waypoints);
        response->message = response->success ? "Waypoints saved." : "Failed to save waypoints.";
      });

    load_srv_ = this->create_service<waypoint_planner::srv::LoadWaypoints>(
      "/waypoint/load",
      [this](
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<waypoint_planner::srv::LoadWaypoints::Request>,
        const std::shared_ptr<waypoint_planner::srv::LoadWaypoints::Response> response)
      {
        response->success = load_waypoints(response->waypoints.waypoints);
        response->message = response->success ? "Waypoints loaded." : "Failed to load waypoints.";
      });

    if (this->get_parameter("auto_load_on_start").as_bool()) {
      const auto waypoint_file = this->get_parameter("waypoints_file").as_string();
      if (std::filesystem::exists(waypoint_file)) {
        std::vector<waypoint_planner::msg::Waypoint> initial_waypoints;
        if (load_waypoints(initial_waypoints)) {
          RCLCPP_INFO(
            this->get_logger(),
            "Loaded %zu waypoints on startup.",
            initial_waypoints.size());
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "Waypoint file exists but could not be loaded: %s",
            waypoint_file.c_str());
        }
      }
    }
  }

private:
  bool save_waypoints(const std::vector<waypoint_planner::msg::Waypoint> & waypoints)
  {
    std::vector<warehouse_utils::Waypoint> converted;
    converted.reserve(waypoints.size());
    const auto frame_id = this->get_parameter("map_frame").as_string();
    for (const auto & wp_msg : waypoints) {
      warehouse_utils::Waypoint wp;
      wp.pose.header.frame_id = frame_id;
      wp.pose.header.stamp = this->now();
      wp.pose.pose = wp_msg.pose;
      wp.hover_time = wp_msg.hover_time;
      wp.acceptance_radius = wp_msg.acceptance_radius;
      converted.push_back(wp);
    }
    return warehouse_utils::save_waypoints(
      this->get_parameter("waypoints_file").as_string(), converted);
  }

  bool load_waypoints(std::vector<waypoint_planner::msg::Waypoint> & output)
  {
    const auto loaded = warehouse_utils::load_waypoints(
      this->get_parameter("waypoints_file").as_string());
    if (loaded.empty()) {
      return false;
    }

    output.clear();
    output.reserve(loaded.size());
    for (const auto & wp : loaded) {
      waypoint_planner::msg::Waypoint msg;
      msg.pose = wp.pose.pose;
      msg.hover_time = wp.hover_time;
      msg.acceptance_radius = wp.acceptance_radius;
      output.push_back(msg);
    }
    publish_markers(loaded);
    return true;
  }

  void publish_markers(const std::vector<warehouse_utils::Waypoint> & waypoints)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    const auto frame_id = this->get_parameter("map_frame").as_string();
    const double lifetime = this->get_parameter("marker_lifetime").as_double();

    for (std::size_t i = 0; i < waypoints.size(); ++i) {
      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = frame_id;
      sphere.header.stamp = this->now();
      sphere.ns = "waypoints";
      sphere.id = static_cast<int>(i);
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose = waypoints[i].pose.pose;
      sphere.scale.x = 0.5;
      sphere.scale.y = 0.5;
      sphere.scale.z = 0.5;
      sphere.color.g = 1.0f;
      sphere.color.a = 0.8f;
      if (lifetime > 0.0) {
        sphere.lifetime = rclcpp::Duration::from_seconds(lifetime);
      }
      marker_array.markers.push_back(sphere);

      visualization_msgs::msg::Marker text;
      text.header.frame_id = frame_id;
      text.header.stamp = this->now();
      text.ns = "waypoint_labels";
      text.id = static_cast<int>(i);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = waypoints[i].pose.pose.position.x;
      text.pose.position.y = waypoints[i].pose.pose.position.y;
      text.pose.position.z = waypoints[i].pose.pose.position.z + 1.0;
      text.scale.z = 0.6;
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      text.color.a = 1.0f;
      text.text = "WP" + std::to_string(i + 1);
      if (lifetime > 0.0) {
        text.lifetime = rclcpp::Duration::from_seconds(lifetime);
      }
      marker_array.markers.push_back(text);
    }

    marker_pub_->publish(marker_array);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<waypoint_planner::srv::SaveWaypoints>::SharedPtr save_srv_;
  rclcpp::Service<waypoint_planner::srv::LoadWaypoints>::SharedPtr load_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManager>());
  rclcpp::shutdown();
  return 0;
}
