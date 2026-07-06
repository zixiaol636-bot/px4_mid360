/**
 * @file mission_executor.cpp
 * @brief Stateful waypoint mission execution with pause, abort and RTL support.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <warehouse_utils/yaml_utils.h>
#include <waypoint_planner/msg/waypoint.hpp>

class MissionExecutor : public rclcpp::Node
{
public:
  MissionExecutor()
  : Node("mission_executor"),
    state_(State::IDLE),
    prev_state_(State::IDLE),
    current_wp_index_(0),
    hover_start_time_(this->now()),
    navigate_start_time_(this->now()),
    paused_at_(this->now()),
    takeoff_altitude_(5.0),
    waypoint_arrival_tolerance_(1.0),
    hover_duration_default_(3.0),
    max_navigate_timeout_(120.0),
    rtl_cruise_altitude_(0.0),
    odom_ready_(false),
    home_pose_initialized_(false)
  {
    using namespace std::placeholders;

    this->declare_parameter<std::string>("waypoints_file", "waypoints.yaml");
    this->declare_parameter<double>("takeoff_altitude", 5.0);
    this->declare_parameter<double>("waypoint_arrival_tolerance", 1.0);
    this->declare_parameter<double>("hover_duration_default", 3.0);
    this->declare_parameter<double>("max_navigate_timeout", 120.0);
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<double>("control_rate", 20.0);

    takeoff_altitude_ = this->get_parameter("takeoff_altitude").as_double();
    waypoint_arrival_tolerance_ =
      this->get_parameter("waypoint_arrival_tolerance").as_double();
    hover_duration_default_ =
      this->get_parameter("hover_duration_default").as_double();
    max_navigate_timeout_ =
      this->get_parameter("max_navigate_timeout").as_double();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_filtered",
      rclcpp::QoS(10),
      std::bind(&MissionExecutor::odom_callback, this, _1));

    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      this->get_parameter("cmd_vel_topic").as_string(), rclcpp::QoS(10));
    state_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/mission/state", rclcpp::QoS(10));

    start_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/mission/start",
      std::bind(&MissionExecutor::handle_start, this, _1, _2, _3));
    pause_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/mission/pause",
      std::bind(&MissionExecutor::handle_pause, this, _1, _2, _3));
    abort_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/mission/abort",
      std::bind(&MissionExecutor::handle_abort, this, _1, _2, _3));

    const double rate = this->get_parameter("control_rate").as_double();
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&MissionExecutor::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "MissionExecutor ready. Initial state: IDLE");
  }

private:
  enum class State
  {
    IDLE,
    TAKEOFF,
    NAVIGATE,
    HOVER_AT_WP,
    RTL,
    PAUSED
  };

  static const char * state_name(State state)
  {
    switch (state) {
      case State::IDLE:
        return "IDLE";
      case State::TAKEOFF:
        return "TAKEOFF";
      case State::NAVIGATE:
        return "NAVIGATE";
      case State::HOVER_AT_WP:
        return "HOVER_AT_WP";
      case State::RTL:
        return "RTL";
      case State::PAUSED:
        return "PAUSED";
      default:
        return "UNKNOWN";
    }
  }

  void handle_start(
    const std::shared_ptr<rmw_request_id_t> /*req_hdr*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (state_ != State::IDLE) {
      response->success = false;
      response->message = "Mission is already active.";
      return;
    }

    if (!odom_ready_) {
      response->success = false;
      response->message = "No odometry received yet.";
      return;
    }

    if (!load_waypoints_from_file()) {
      response->success = false;
      response->message = "Failed to load waypoint file.";
      return;
    }

    home_pose_ = current_odom_.pose.pose;
    home_pose_initialized_ = true;
    current_wp_index_ = 0;
    hover_start_time_ = this->now();
    navigate_start_time_ = this->now();

    transition_to(State::TAKEOFF, "mission start");
    response->success = true;
    response->message = "Mission started.";
  }

  void handle_pause(
    const std::shared_ptr<rmw_request_id_t> /*req_hdr*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (state_ == State::PAUSED) {
      const auto pause_duration = this->now() - paused_at_;
      if (prev_state_ == State::HOVER_AT_WP) {
        hover_start_time_ = hover_start_time_ + pause_duration;
      } else if (prev_state_ == State::NAVIGATE) {
        navigate_start_time_ = navigate_start_time_ + pause_duration;
      }

      transition_to(prev_state_, "mission resume");
      response->success = true;
      response->message = "Mission resumed.";
      return;
    }

    if (state_ == State::IDLE || state_ == State::RTL) {
      response->success = false;
      response->message = "Current state cannot be paused.";
      return;
    }

    prev_state_ = state_;
    paused_at_ = this->now();
    transition_to(State::PAUSED, "mission pause");
    response->success = true;
    response->message = "Mission paused.";
  }

  void handle_abort(
    const std::shared_ptr<rmw_request_id_t> /*req_hdr*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (state_ == State::IDLE) {
      response->success = false;
      response->message = "Mission is not running.";
      return;
    }

    enter_rtl("mission abort");
    response->success = true;
    response->message = "Mission aborted. Returning home.";
  }

  void control_loop()
  {
    publish_state();

    if (!odom_ready_ && state_ != State::IDLE) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Mission active but odometry is not available yet.");
      send_zero_velocity();
      return;
    }

    switch (state_) {
      case State::IDLE:
        break;
      case State::TAKEOFF:
        execute_takeoff();
        break;
      case State::NAVIGATE:
        execute_navigate();
        break;
      case State::HOVER_AT_WP:
        execute_hover();
        break;
      case State::RTL:
        execute_rtl();
        break;
      case State::PAUSED:
        send_zero_velocity();
        break;
    }
  }

  void execute_takeoff()
  {
    if (!home_pose_initialized_) {
      RCLCPP_ERROR(this->get_logger(), "Cannot take off without a valid home pose.");
      transition_to(State::IDLE, "missing home pose");
      return;
    }

    const double target_altitude = home_pose_.position.z + takeoff_altitude_;
    const double error_z = target_altitude - current_odom_.pose.pose.position.z;
    const double climb_rate = std::clamp(error_z * 0.5, -0.5, 1.0);

    if (std::abs(error_z) < 0.3) {
      navigate_start_time_ = this->now();
      transition_to(State::NAVIGATE, "takeoff complete");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.z = climb_rate;
    cmd_vel_pub_->publish(cmd);
  }

  void execute_navigate()
  {
    if (waypoints_.empty() || current_wp_index_ >= waypoints_.size()) {
      enter_rtl("mission complete");
      return;
    }

    const auto & wp = waypoints_[current_wp_index_];
    const double dx = wp.pose.position.x - current_odom_.pose.pose.position.x;
    const double dy = wp.pose.position.y - current_odom_.pose.pose.position.y;
    const double dz = wp.pose.position.z - current_odom_.pose.pose.position.z;
    const double dist_xy = std::hypot(dx, dy);
    const double acceptance =
      wp.acceptance_radius > 0.0 ? wp.acceptance_radius : waypoint_arrival_tolerance_;

    if (dist_xy < acceptance && std::abs(dz) < acceptance) {
      hover_start_time_ = this->now();
      transition_to(State::HOVER_AT_WP, "waypoint reached");
      return;
    }

    const double elapsed = (this->now() - navigate_start_time_).seconds();
    if (max_navigate_timeout_ > 0.0 && elapsed > max_navigate_timeout_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Navigation to waypoint %zu timed out after %.1f seconds. Returning home.",
        current_wp_index_ + 1,
        elapsed);
      enter_rtl("navigation timeout");
      return;
    }

    send_velocity_toward_pose(wp.pose, 2.0, 1.0, 0.5);
  }

  void execute_hover()
  {
    if (current_wp_index_ >= waypoints_.size()) {
      enter_rtl("hover state without waypoint");
      return;
    }

    const auto & wp = waypoints_[current_wp_index_];
    const double hover_duration =
      wp.hover_time > 0.0 ? wp.hover_time : hover_duration_default_;
    const double elapsed = (this->now() - hover_start_time_).seconds();

    if (elapsed >= hover_duration) {
      ++current_wp_index_;
      if (current_wp_index_ < waypoints_.size()) {
        navigate_start_time_ = this->now();
        transition_to(State::NAVIGATE, "hover complete");
      } else {
        enter_rtl("final waypoint complete");
      }
      return;
    }

    send_velocity_toward_pose(wp.pose, 0.5, 0.3, 0.4);
  }

  void execute_rtl()
  {
    if (!home_pose_initialized_) {
      RCLCPP_ERROR(this->get_logger(), "RTL requested without a stored home pose.");
      transition_to(State::IDLE, "missing home pose for RTL");
      return;
    }

    const double dx = home_pose_.position.x - current_odom_.pose.pose.position.x;
    const double dy = home_pose_.position.y - current_odom_.pose.pose.position.y;
    const double dist_xy = std::hypot(dx, dy);

    if (dist_xy > 0.5) {
      auto rtl_pose = home_pose_;
      rtl_pose.position.z = rtl_cruise_altitude_;
      send_velocity_toward_pose(rtl_pose, 1.5, 0.8, 0.5);
      return;
    }

    const double landing_target_z = home_pose_.position.z + 0.2;
    const double dz = landing_target_z - current_odom_.pose.pose.position.z;

    if (std::abs(dz) < 0.15) {
      send_zero_velocity();
      transition_to(State::IDLE, "RTL complete");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.z = std::clamp(dz * 0.3, -0.5, 0.3);
    cmd_vel_pub_->publish(cmd);
  }

  void enter_rtl(const std::string & reason)
  {
    if (!home_pose_initialized_) {
      transition_to(State::IDLE, "cannot enter RTL without home pose");
      return;
    }

    rtl_cruise_altitude_ = std::max(
      current_odom_.pose.pose.position.z,
      home_pose_.position.z + takeoff_altitude_);
    transition_to(State::RTL, reason);
  }

  void send_velocity_toward_pose(
    const geometry_msgs::msg::Pose & target_pose,
    double max_xy_speed,
    double max_z_speed,
    double max_yaw_rate)
  {
    const double dx = target_pose.position.x - current_odom_.pose.pose.position.x;
    const double dy = target_pose.position.y - current_odom_.pose.pose.position.y;
    const double dz = target_pose.position.z - current_odom_.pose.pose.position.z;

    const double vx_world = std::clamp(dx * 0.5, -max_xy_speed, max_xy_speed);
    const double vy_world = std::clamp(dy * 0.5, -max_xy_speed, max_xy_speed);
    const double vz = std::clamp(dz * 0.5, -max_z_speed, max_z_speed);

    const double current_yaw = get_yaw_from_quat(current_odom_.pose.pose.orientation);
    double desired_yaw = current_yaw;
    if (std::hypot(dx, dy) > 0.1) {
      desired_yaw = std::atan2(dy, dx);
    }

    const double cos_yaw = std::cos(current_yaw);
    const double sin_yaw = std::sin(current_yaw);
    const double vx_body = cos_yaw * vx_world + sin_yaw * vy_world;
    const double vy_body = -sin_yaw * vx_world + cos_yaw * vy_world;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx_body;
    cmd.linear.y = vy_body;
    cmd.linear.z = vz;
    cmd.angular.z = std::clamp(
      shortest_angular_distance(current_yaw, desired_yaw),
      -max_yaw_rate,
      max_yaw_rate);
    cmd_vel_pub_->publish(cmd);
  }

  void transition_to(State next_state, const std::string & reason)
  {
    if (state_ == next_state) {
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Mission state: %s -> %s (%s)",
      state_name(state_),
      state_name(next_state),
      reason.c_str());
    state_ = next_state;
  }

  bool load_waypoints_from_file()
  {
    std::string filepath;
    this->get_parameter("waypoints_file", filepath);

    const auto wps = warehouse_utils::load_waypoints(filepath);
    if (wps.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No waypoints loaded from %s", filepath.c_str());
      return false;
    }

    waypoints_.clear();
    waypoints_.reserve(wps.size());
    for (const auto & wp : wps) {
      waypoint_planner::msg::Waypoint msg;
      msg.pose = wp.pose.pose;
      msg.hover_time = wp.hover_time;
      msg.acceptance_radius = wp.acceptance_radius;
      waypoints_.push_back(msg);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded %zu waypoints from %s",
      waypoints_.size(),
      filepath.c_str());
    return true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_odom_ = *msg;
    odom_ready_ = true;
  }

  static double get_yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
  {
    tf2::Quaternion tf2_q(q.x, q.y, q.z, q.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  static double shortest_angular_distance(double from, double to)
  {
    double diff = to - from;
    while (diff > M_PI) {
      diff -= 2.0 * M_PI;
    }
    while (diff < -M_PI) {
      diff += 2.0 * M_PI;
    }
    return diff;
  }

  void send_zero_velocity()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
  }

  void publish_state()
  {
    std_msgs::msg::String msg;
    msg.data = state_name(state_);
    state_pub_->publish(msg);
  }

  State state_;
  State prev_state_;
  nav_msgs::msg::Odometry current_odom_;
  std::vector<waypoint_planner::msg::Waypoint> waypoints_;
  size_t current_wp_index_;
  rclcpp::Time hover_start_time_;
  rclcpp::Time navigate_start_time_;
  rclcpp::Time paused_at_;
  geometry_msgs::msg::Pose home_pose_;
  double takeoff_altitude_;
  double waypoint_arrival_tolerance_;
  double hover_duration_default_;
  double max_navigate_timeout_;
  double rtl_cruise_altitude_;
  bool odom_ready_;
  bool home_pose_initialized_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExecutor>());
  rclcpp::shutdown();
  return 0;
}
