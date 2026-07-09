#include <memory>
#include <string>

#include <px4_msgs/msg/battery_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <std_msgs/msg/bool.hpp>

using px4_msgs::msg::BatteryStatus;

class BatteryMonitor : public rclcpp::Node
{
public:
  BatteryMonitor() : Node("battery_monitor")
  {
    this->declare_parameter<std::string>("battery_topic", "/fmu/out/battery_status");
    this->declare_parameter<std::string>("px4_namespace", "");
    this->declare_parameter<std::string>("status_topic", "/safety/battery_status");
    this->declare_parameter<double>("low_battery_threshold", 0.25);
    this->declare_parameter<double>("critical_battery_threshold", 0.15);
    this->declare_parameter<double>("voltage_warn", 10.5);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    const auto px4_qos = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, 5),
      qos_profile);
    battery_sub_ = this->create_subscription<BatteryStatus>(
      px4_topic(
        this->get_parameter("px4_namespace").as_string(),
        this->get_parameter("battery_topic").as_string()),
      px4_qos,
      [this](const BatteryStatus::SharedPtr msg) { battery_callback(msg); });

    status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      this->get_parameter("status_topic").as_string(), rclcpp::QoS(1).transient_local());
  }

private:
  static std::string px4_topic(const std::string & px4_namespace, const std::string & topic)
  {
    if (px4_namespace.empty()) {
      return topic;
    }

    std::string prefix = px4_namespace;
    if (prefix.front() != '/') {
      prefix.insert(prefix.begin(), '/');
    }
    while (prefix.size() > 1 && prefix.back() == '/') {
      prefix.pop_back();
    }

    std::string suffix = topic;
    if (suffix.empty()) {
      return prefix;
    }
    if (suffix.front() != '/') {
      suffix.insert(suffix.begin(), '/');
    }
    return prefix + suffix;
  }

  void battery_callback(const BatteryStatus::SharedPtr msg)
  {
    const double low_threshold = this->get_parameter("low_battery_threshold").as_double();
    const double critical_threshold =
      this->get_parameter("critical_battery_threshold").as_double();
    const double voltage_warn = this->get_parameter("voltage_warn").as_double();

    const double percentage = msg->remaining;
    const bool critical =
      (percentage >= 0.0 && percentage <= critical_threshold) ||
      (msg->voltage_v > 0.0 && msg->voltage_v <= voltage_warn);
    const bool low = percentage >= 0.0 && percentage <= low_threshold;
    const bool healthy = msg->connected && !(critical || low);

    if (!has_published_status_ || healthy != last_published_healthy_) {
      std_msgs::msg::Bool status;
      status.data = healthy;
      status_pub_->publish(status);
      last_published_healthy_ = healthy;
      has_published_status_ = true;
    }

    if (critical) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "Battery critical: %.1f%% %.2fV", percentage * 100.0, msg->voltage_v);
    } else if (low) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000,
        "Battery low: %.1f%% %.2fV", percentage * 100.0, msg->voltage_v);
    }
  }

  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  bool last_published_healthy_{true};
  bool has_published_status_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryMonitor>());
  rclcpp::shutdown();
  return 0;
}
