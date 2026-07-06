#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

class DriverMonitor : public rclcpp::Node
{
public:
  DriverMonitor() : Node("driver_monitor"), last_msg_time_(this->now())
  {
    this->declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    this->declare_parameter<double>("watchdog_timeout", 2.0);
    this->declare_parameter<double>("check_rate", 10.0);
    this->declare_parameter<bool>("publish_health", true);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      this->get_parameter("lidar_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr) { last_msg_time_ = this->now(); });

    if (this->get_parameter("publish_health").as_bool()) {
      health_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/driver/health", rclcpp::QoS(1).transient_local());
    }

    const double check_rate = this->get_parameter("check_rate").as_double();
    watchdog_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / check_rate),
      [this]() { watchdog_check(); });
  }

private:
  void watchdog_check()
  {
    if (!health_pub_) {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data =
      (this->now() - last_msg_time_).seconds() <= this->get_parameter("watchdog_timeout").as_double();
    health_pub_->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_msg_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriverMonitor>());
  rclcpp::shutdown();
  return 0;
}
