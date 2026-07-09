#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class GeofenceMonitor : public rclcpp::Node
{
public:
  GeofenceMonitor() : Node("geofence_monitor")
  {
    this->declare_parameter<std::string>("odom_topic", "/odom_filtered");
    this->declare_parameter<std::string>("status_topic", "/safety/geofence_status");
    this->declare_parameter<std::vector<double>>(
      "geofence_vertices",
      {-50.0, -50.0, 50.0, -50.0, 50.0, 50.0, -50.0, 50.0});

    const auto raw_polygon = this->get_parameter("geofence_vertices").as_double_array();
    for (std::size_t i = 0; i + 1 < raw_polygon.size(); i += 2) {
      polygon_.emplace_back(raw_polygon[i], raw_polygon[i + 1]);
    }

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("odom_topic").as_string(), rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });

    status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      this->get_parameter("status_topic").as_string(), rclcpp::QoS(1).transient_local());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & position = msg->pose.pose.position;
    const bool inside = point_in_polygon(position.x, position.y);
    if (!has_published_status_ || inside != last_inside_state_) {
      std_msgs::msg::Bool status;
      status.data = inside;
      status_pub_->publish(status);
      last_inside_state_ = inside;
      has_published_status_ = true;
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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  std::vector<std::pair<double, double>> polygon_;
  bool last_inside_state_{true};
  bool has_published_status_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeofenceMonitor>());
  rclcpp::shutdown();
  return 0;
}
