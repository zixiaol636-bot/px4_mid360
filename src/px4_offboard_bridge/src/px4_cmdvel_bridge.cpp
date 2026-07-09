#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;

class Px4CmdvelBridge : public rclcpp::Node
{
public:
  Px4CmdvelBridge()
  : Node("px4_cmdvel_bridge")
  {
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
    declare_parameter<std::string>("odom_topic", "/odom_filtered");
    declare_parameter<std::string>("px4_namespace", "");
    declare_parameter<std::string>("offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    declare_parameter<std::string>("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    declare_parameter<std::string>("vehicle_command_topic", "/fmu/in/vehicle_command");
    declare_parameter<double>("publish_rate", 25.0);
    declare_parameter<double>("cmd_vel_timeout", 0.5);
    declare_parameter<double>("max_vel_horizontal", 1.5);
    declare_parameter<double>("max_vel_vertical", 0.5);
    declare_parameter<double>("max_yaw_rate", 1.0);
    declare_parameter<bool>("auto_offboard", false);
    declare_parameter<bool>("auto_arm", false);
    declare_parameter<int>("startup_setpoint_count", 10);

    const auto cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
    const auto odom_topic = get_parameter("odom_topic").as_string();
    const auto px4_namespace = get_parameter("px4_namespace").as_string();
    const double publish_rate = get_parameter("publish_rate").as_double();

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_cmd_ = *msg;
        last_cmd_time_ = now();
        has_cmd_ = true;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        has_odom_ = true;
      });

    offboard_mode_pub_ = create_publisher<OffboardControlMode>(
      px4_topic(px4_namespace, get_parameter("offboard_control_mode_topic").as_string()),
      rclcpp::QoS(10));
    trajectory_pub_ = create_publisher<TrajectorySetpoint>(
      px4_topic(px4_namespace, get_parameter("trajectory_setpoint_topic").as_string()),
      rclcpp::QoS(10));
    vehicle_command_pub_ = create_publisher<VehicleCommand>(
      px4_topic(px4_namespace, get_parameter("vehicle_command_topic").as_string()),
      rclcpp::QoS(10));

    arm_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/arm",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
        response->success = true;
        response->message = "PX4 arm command sent.";
      });

    disarm_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/disarm",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
        response->success = true;
        response->message = "PX4 disarm command sent.";
      });

    offboard_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/set_offboard",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
        response->success = true;
        response->message = "PX4 offboard mode command sent.";
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      [this]() { publish_setpoint(); });
  }

private:
  uint64_t timestamp_us() const
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000ULL);
  }

  static double yaw_from_odom(const nav_msgs::msg::Odometry & odom)
  {
    tf2::Quaternion q;
    tf2::fromMsg(odom.pose.pose.orientation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  static float nan_f()
  {
    return std::numeric_limits<float>::quiet_NaN();
  }

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

  void publish_setpoint()
  {
    const auto stamp = timestamp_us();

    OffboardControlMode mode{};
    mode.timestamp = stamp;
    mode.position = false;
    mode.velocity = true;
    mode.acceleration = false;
    mode.attitude = false;
    mode.body_rate = false;
    offboard_mode_pub_->publish(mode);

    TrajectorySetpoint setpoint{};
    setpoint.timestamp = stamp;
    setpoint.position = {nan_f(), nan_f(), nan_f()};
    setpoint.velocity = {0.0F, 0.0F, 0.0F};
    setpoint.acceleration = {nan_f(), nan_f(), nan_f()};
    setpoint.jerk = {nan_f(), nan_f(), nan_f()};
    setpoint.yaw = nan_f();
    setpoint.yawspeed = 0.0F;

    geometry_msgs::msg::Twist cmd;
    const bool fresh_cmd = has_cmd_ &&
      (now() - last_cmd_time_).seconds() <= get_parameter("cmd_vel_timeout").as_double();
    if (fresh_cmd) {
      cmd = latest_cmd_;
    }

    const double max_horizontal = get_parameter("max_vel_horizontal").as_double();
    const double max_vertical = get_parameter("max_vel_vertical").as_double();
    const double max_yaw_rate = get_parameter("max_yaw_rate").as_double();

    const double vx_body = std::clamp(cmd.linear.x, -max_horizontal, max_horizontal);
    const double vy_body = std::clamp(cmd.linear.y, -max_horizontal, max_horizontal);
    const double vz_enu = std::clamp(cmd.linear.z, -max_vertical, max_vertical);
    const double yaw_rate_enu = std::clamp(cmd.angular.z, -max_yaw_rate, max_yaw_rate);

    double vx_enu = vx_body;
    double vy_enu = vy_body;
    if (has_odom_) {
      const double yaw = yaw_from_odom(latest_odom_);
      vx_enu = std::cos(yaw) * vx_body - std::sin(yaw) * vy_body;
      vy_enu = std::sin(yaw) * vx_body + std::cos(yaw) * vy_body;
    }

    setpoint.velocity = {
      static_cast<float>(vy_enu),
      static_cast<float>(vx_enu),
      static_cast<float>(-vz_enu)
    };
    setpoint.yawspeed = static_cast<float>(-yaw_rate_enu);
    trajectory_pub_->publish(setpoint);

    const int warmup_count = get_parameter("startup_setpoint_count").as_int();
    if (setpoint_counter_ == warmup_count) {
      if (get_parameter("auto_offboard").as_bool()) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
      }
      if (get_parameter("auto_arm").as_bool()) {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
      }
    }
    if (setpoint_counter_ <= warmup_count) {
      ++setpoint_counter_;
    }
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.0F, float param2 = 0.0F)
  {
    VehicleCommand msg{};
    msg.timestamp = timestamp_us();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  geometry_msgs::msg::Twist latest_cmd_;
  nav_msgs::msg::Odometry latest_odom_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool has_cmd_{false};
  bool has_odom_{false};
  int setpoint_counter_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr offboard_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4CmdvelBridge>());
  rclcpp::shutdown();
  return 0;
}
