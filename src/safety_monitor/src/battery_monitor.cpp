#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>

class BatteryMonitor : public rclcpp::Node
{
public:
  BatteryMonitor() : Node("battery_monitor")
  {
    this->declare_parameter<std::string>("battery_topic", "/mavros/battery");
    this->declare_parameter<std::string>("status_topic", "/safety/battery_status");
    this->declare_parameter<double>("low_battery_threshold", 0.25);
    this->declare_parameter<double>("critical_battery_threshold", 0.15);
    this->declare_parameter<double>("voltage_warn", 10.5);

    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      this->get_parameter("battery_topic").as_string(), rclcpp::QoS(10),
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) { battery_callback(msg); });

    status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      this->get_parameter("status_topic").as_string(), rclcpp::QoS(1).transient_local());
  }

private:
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    const double low_threshold = this->get_parameter("low_battery_threshold").as_double();
    const double critical_threshold =
      this->get_parameter("critical_battery_threshold").as_double();
    const double voltage_warn = this->get_parameter("voltage_warn").as_double();

    const double percentage = msg->percentage;
    const bool critical =
      (percentage >= 0.0 && percentage <= critical_threshold) ||
      (msg->voltage > 0.0 && msg->voltage <= voltage_warn);
    const bool low = percentage >= 0.0 && percentage <= low_threshold;
    const bool healthy = !(critical || low);

    if (healthy != last_published_healthy_) {
      std_msgs::msg::Bool status;
      status.data = healthy;
      status_pub_->publish(status);
      last_published_healthy_ = healthy;
    }

    if (critical) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "Battery critical: %.1f%% %.2fV", percentage * 100.0, msg->voltage);
    } else if (low) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "Battery low: %.1f%% %.2fV", percentage * 100.0, msg->voltage);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  bool last_published_healthy_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryMonitor>());
  rclcpp::shutdown();
  return 0;
}
