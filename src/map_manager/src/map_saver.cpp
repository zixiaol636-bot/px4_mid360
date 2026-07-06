#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using PointType = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointType>;

class MapSaver : public rclcpp::Node
{
public:
  MapSaver() : Node("map_saver"), global_map_(std::make_shared<CloudT>())
  {
    this->declare_parameter<std::string>("map_file", "");
    this->declare_parameter<std::string>("save_directory", "/tmp");
    this->declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    this->declare_parameter<std::string>("odom_topic", "/Odometry");

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      this->get_parameter("cloud_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud_callback(msg); });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->get_parameter("odom_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = *msg;
        has_odom_ = true;
      });

    save_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/map/save",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        response->success = save_map();
        response->message = response->success ? "Map saved." : "Failed to save map.";
      });
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    nav_msgs::msg::Odometry odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!has_odom_) {
        return;
      }
      odom = latest_odom_;
    }

    CloudT cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty()) {
      return;
    }

    tf2::Quaternion q(
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z,
      odom.pose.pose.orientation.w);
    tf2::Matrix3x3 rotation(q);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        transform(row, col) = static_cast<float>(rotation[row][col]);
      }
    }
    transform(0, 3) = static_cast<float>(odom.pose.pose.position.x);
    transform(1, 3) = static_cast<float>(odom.pose.pose.position.y);
    transform(2, 3) = static_cast<float>(odom.pose.pose.position.z);

    CloudT transformed;
    pcl::transformPointCloud(cloud, transformed, transform);

    std::lock_guard<std::mutex> lock(map_mutex_);
    *global_map_ += transformed;
  }

  bool save_map()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (global_map_->empty()) {
      RCLCPP_WARN(this->get_logger(), "Global map is empty, nothing to save.");
      return false;
    }

    std::string filepath = this->get_parameter("map_file").as_string();
    if (filepath.empty()) {
      filepath = this->get_parameter("save_directory").as_string() + "/map_" +
        std::to_string(this->now().nanoseconds()) + ".pcd";
    }

    if (pcl::io::savePCDFileBinary(filepath, *global_map_) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save PCD map: %s", filepath.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Saved map to %s", filepath.c_str());
    return true;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;

  std::mutex odom_mutex_;
  std::mutex map_mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  bool has_odom_{false};
  CloudT::Ptr global_map_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapSaver>());
  rclcpp::shutdown();
  return 0;
}
