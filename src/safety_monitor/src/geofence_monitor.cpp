#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class GeofenceMonitor : public rclcpp::Node
{
public:
  GeofenceMonitor() : Node("geofence_monitor")
  {
    this->declare_parameter<std::string>("pose_topic", "/mavros/local_position/pose");
    this->declare_parameter<std::string>("status_topic", "/safety/geofence_status");
    this->declare_parameter<std::string>(
      "cmd_vel_topic", "/mavros/setpoint_velocity/cmd_vel");
    this->declare_parameter<std::vector<double>>(
      "geofence_vertices",
      {-50.0, -50.0, 50.0, -50.0, 50.0, 50.0, -50.0, 50.0});
    this->declare_parameter<bool>("enforce_hold", true);
    this->declare_parameter<double>("hold_altitude", 5.0);

    const auto raw_polygon = this->get_parameter("geofence_vertices").as_double_array();
    for (std::size_t i = 0; i + 1 < raw_polygon.size(); i += 2) {
      polygon_.emplace_back(raw_polygon[i], raw_polygon[i + 1]);
    }

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      this->get_parameter("pose_topic").as_string(), rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_callback(msg); });

    status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      this->get_parameter("status_topic").as_string(), rclcpp::QoS(1).transient_local());
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      this->get_parameter("cmd_vel_topic").as_string(), rclcpp::QoS(10));
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const bool inside = point_in_polygon(msg->pose.position.x, msg->pose.position.y);
    if (inside != last_inside_state_) {
      std_msgs::msg::Bool status;
      status.data = inside;
      status_pub_->publish(status);
      last_inside_state_ = inside;
    }

    if (!inside && this->get_parameter("enforce_hold").as_bool()) {
      geometry_msgs::msg::TwistStamped hold_msg;
      hold_msg.header.stamp = this->now();
      hold_msg.header.frame_id = "base_link";
      const double altitude_error =
        this->get_parameter("hold_altitude").as_double() - msg->pose.position.z;
      hold_msg.twist.linear.z = std::clamp(altitude_error * 0.3, -0.5, 0.5);
      cmd_vel_pub_->publish(hold_msg);
    }
  }

  bool point_in_polygon(double x, double y) const
  {
    if (polygon_.size() < 3) {
      return true;
    }

    bool inside = false;
    for (std::size_t i = 0, j = polygon_.size() - 1; i < polygon_.size(); j = i++) {
      const auto & a = polygon_[i];
      const auto & b = polygon_[j];
      if (((a.second > y) != (b.second > y)) &&
          (x < (b.first - a.first) * (y - a.second) / (b.second - a.second) + a.first))
      {
        inside = !inside;
      }
    }
    return inside;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  std::vector<std::pair<double, double>> polygon_;
  bool last_inside_state_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeofenceMonitor>());
  rclcpp::shutdown();
  return 0;
}
