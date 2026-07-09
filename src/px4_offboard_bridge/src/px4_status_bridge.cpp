#include <memory>
#include <sstream>
#include <string>

#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleStatus;

class Px4StatusBridge : public rclcpp::Node
{
public:
  Px4StatusBridge()
  : Node("px4_status_bridge")
  {
    declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status");
    declare_parameter<std::string>("vehicle_command_ack_topic", "/fmu/out/vehicle_command_ack");
    declare_parameter<std::string>("px4_namespace", "");
    declare_parameter<std::string>("status_text_topic", "/px4_native/status");
    declare_parameter<std::string>("armed_topic", "/px4_native/armed");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    const auto px4_qos = rclcpp::QoS(
      rclcpp::QoSInitialization(qos_profile.history, 5),
      qos_profile);
    const auto px4_namespace = get_parameter("px4_namespace").as_string();

    status_sub_ = create_subscription<VehicleStatus>(
      px4_topic(px4_namespace, get_parameter("vehicle_status_topic").as_string()), px4_qos,
      [this](const VehicleStatus::SharedPtr msg) {
        latest_status_ = *msg;
        has_status_ = true;
        publish_status();
      });

    ack_sub_ = create_subscription<VehicleCommandAck>(
      px4_topic(px4_namespace, get_parameter("vehicle_command_ack_topic").as_string()), px4_qos,
      [this](const VehicleCommandAck::SharedPtr msg) {
        latest_ack_ = *msg;
        has_ack_ = true;
        publish_status();
      });

    status_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_text_topic").as_string(), rclcpp::QoS(10));
    armed_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("armed_topic").as_string(), rclcpp::QoS(10));
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

  void publish_status()
  {
    std_msgs::msg::String text;
    std::ostringstream stream;

    if (has_status_) {
      stream << "arming_state=" << static_cast<int>(latest_status_.arming_state)
             << " nav_state=" << static_cast<int>(latest_status_.nav_state)
             << " failsafe=" << static_cast<int>(latest_status_.failsafe);

      std_msgs::msg::Bool armed;
      armed.data = latest_status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
      armed_pub_->publish(armed);
    } else {
      stream << "waiting_for_vehicle_status";
    }

    if (has_ack_) {
      stream << " last_command=" << latest_ack_.command
             << " result=" << static_cast<int>(latest_ack_.result);
    }

    text.data = stream.str();
    status_pub_->publish(text);
  }

  VehicleStatus latest_status_;
  VehicleCommandAck latest_ack_;
  bool has_status_{false};
  bool has_ack_{false};

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4StatusBridge>());
  rclcpp::shutdown();
  return 0;
}
