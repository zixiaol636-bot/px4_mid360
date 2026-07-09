#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using px4_msgs::msg::VehicleOdometry;

class Px4VisualOdometryBridge : public rclcpp::Node
{
public:
  Px4VisualOdometryBridge()
  : Node("px4_visual_odometry_bridge")
  {
    declare_parameter<std::string>("odom_topic", "/odom_filtered");
    declare_parameter<std::string>("px4_namespace", "");
    declare_parameter<std::string>("vehicle_visual_odometry_topic", "/fmu/in/vehicle_visual_odometry");
    declare_parameter<double>("publish_rate", 30.0);
    declare_parameter<double>("position_variance", 0.03);
    declare_parameter<double>("orientation_variance", 0.05);
    declare_parameter<double>("velocity_variance", 0.08);
    declare_parameter<bool>("use_twist_velocity", false);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        has_odom_ = true;
      });

    odom_pub_ = create_publisher<VehicleOdometry>(
      px4_topic(
        get_parameter("px4_namespace").as_string(),
        get_parameter("vehicle_visual_odometry_topic").as_string()),
      rclcpp::QoS(10));

    const double publish_rate = get_parameter("publish_rate").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      [this]() { publish_visual_odometry(); });
  }

private:
  uint64_t timestamp_us() const
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000ULL);
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

  static tf2::Quaternion enu_flu_to_ned_frd(const geometry_msgs::msg::Quaternion & q_msg)
  {
    tf2::Quaternion q_enu_flu;
    tf2::fromMsg(q_msg, q_enu_flu);
    tf2::Matrix3x3 r_enu_flu(q_enu_flu);
    tf2::Matrix3x3 r_ned_enu(
      0.0, 1.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 0.0, -1.0);
    tf2::Matrix3x3 r_flu_frd(
      1.0, 0.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0);
    tf2::Matrix3x3 r_ned_frd = r_ned_enu * r_enu_flu * r_flu_frd;
    tf2::Quaternion q_ned_frd;
    r_ned_frd.getRotation(q_ned_frd);
    q_ned_frd.normalize();
    return q_ned_frd;
  }

  void publish_visual_odometry()
  {
    if (!has_odom_) {
      return;
    }

    const auto & pose = latest_odom_.pose.pose;
    const auto & twist = latest_odom_.twist.twist;
    const double position_var = get_parameter("position_variance").as_double();
    const double orientation_var = get_parameter("orientation_variance").as_double();
    const double velocity_var = get_parameter("velocity_variance").as_double();

    VehicleOdometry msg{};
    msg.timestamp = timestamp_us();
    msg.timestamp_sample = msg.timestamp;
    msg.pose_frame = VehicleOdometry::POSE_FRAME_NED;
    msg.position = {
      static_cast<float>(pose.position.y),
      static_cast<float>(pose.position.x),
      static_cast<float>(-pose.position.z)
    };

    const tf2::Quaternion q_ned_frd = enu_flu_to_ned_frd(pose.orientation);
    msg.q = {
      static_cast<float>(q_ned_frd.w()),
      static_cast<float>(q_ned_frd.x()),
      static_cast<float>(q_ned_frd.y()),
      static_cast<float>(q_ned_frd.z())
    };

    msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;
    if (get_parameter("use_twist_velocity").as_bool()) {
      msg.velocity = {
        static_cast<float>(twist.linear.y),
        static_cast<float>(twist.linear.x),
        static_cast<float>(-twist.linear.z)
      };
    } else {
      msg.velocity = {nan_f(), nan_f(), nan_f()};
    }
    msg.angular_velocity = {nan_f(), nan_f(), nan_f()};
    msg.position_variance = {
      static_cast<float>(position_var),
      static_cast<float>(position_var),
      static_cast<float>(position_var)
    };
    msg.orientation_variance = {
      static_cast<float>(orientation_var),
      static_cast<float>(orientation_var),
      static_cast<float>(orientation_var)
    };
    msg.velocity_variance = {
      static_cast<float>(velocity_var),
      static_cast<float>(velocity_var),
      static_cast<float>(velocity_var)
    };
    msg.reset_counter = 0;
    msg.quality = 100;
    odom_pub_->publish(msg);
  }

  nav_msgs::msg::Odometry latest_odom_;
  bool has_odom_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4VisualOdometryBridge>());
  rclcpp::shutdown();
  return 0;
}
