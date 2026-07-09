#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using PointType = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointType>;

class CloudPreprocessor : public rclcpp::Node
{
public:
  CloudPreprocessor() : Node("cloud_preprocessor")
  {
    declare_parameter<std::string>("cloud_topic_in", "/cloud_registered");
    declare_parameter<std::string>("cloud_topic_out", "/cloud_local_obstacles");
    declare_parameter<std::string>("odom_topic", "/odom_filtered");
    declare_parameter<std::string>("pointcloud_frame_mode", "world");
    declare_parameter<std::string>("output_frame_id", "");
    declare_parameter<bool>("require_odom_for_world_crop", true);

    declare_parameter<bool>("enable_local_crop", true);
    declare_parameter<double>("crop_min_x", -8.0);
    declare_parameter<double>("crop_max_x", 8.0);
    declare_parameter<double>("crop_min_y", -8.0);
    declare_parameter<double>("crop_max_y", 8.0);
    declare_parameter<double>("crop_min_z", -2.0);
    declare_parameter<double>("crop_max_z", 2.0);

    declare_parameter<bool>("enable_voxel_filter", true);
    declare_parameter<double>("voxel_leaf_size", 0.12);
    declare_parameter<std::string>("outlier_filter_type", "none");
    declare_parameter<int>("statistical_mean_k", 20);
    declare_parameter<double>("statistical_stddev_mul", 1.0);
    declare_parameter<double>("radius_search", 0.25);
    declare_parameter<int>("radius_min_neighbors", 2);
    declare_parameter<bool>("publish_empty_on_failure", true);

    const auto cloud_topic_in = get_parameter("cloud_topic_in").as_string();
    const auto cloud_topic_out = get_parameter("cloud_topic_out").as_string();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_in, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_callback(msg);
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        has_odom_ = true;
      });

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_topic_out, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "CloudPreprocessor ready: %s -> %s. Raw cloud is preserved for mapping/inventory.",
      cloud_topic_in.c_str(),
      cloud_topic_out.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    CloudT::Ptr cloud(new CloudT());
    pcl::fromROSMsg(*msg, *cloud);
    remove_invalid_points(cloud);

    if (cloud->empty()) {
      publish_cloud(*cloud, msg->header);
      return;
    }

    if (get_parameter("enable_local_crop").as_bool()) {
      const bool cropped = crop_local_obstacle_region(cloud);
      if (!cropped) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Skipping obstacle cloud publish because odometry is required for world-frame crop.");
        if (get_parameter("publish_empty_on_failure").as_bool()) {
          CloudT empty;
          publish_cloud(empty, msg->header);
        }
        return;
      }
    }

    if (get_parameter("enable_voxel_filter").as_bool()) {
      apply_voxel_filter(cloud);
    }

    apply_outlier_filter(cloud);
    publish_cloud(*cloud, msg->header);
  }

  void remove_invalid_points(CloudT::Ptr cloud) const
  {
    CloudT filtered;
    filtered.header = cloud->header;
    filtered.points.reserve(cloud->points.size());
    for (const auto & point : cloud->points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        filtered.points.push_back(point);
      }
    }
    filtered.width = static_cast<std::uint32_t>(filtered.points.size());
    filtered.height = 1;
    filtered.is_dense = true;
    *cloud = filtered;
  }

  bool crop_local_obstacle_region(CloudT::Ptr cloud) const
  {
    const auto frame_mode = get_parameter("pointcloud_frame_mode").as_string();
    const bool world_frame = frame_mode != "body";
    if (world_frame && !has_odom_) {
      return !get_parameter("require_odom_for_world_crop").as_bool();
    }

    tf2::Quaternion q_body_to_world;
    tf2::Vector3 origin(0.0, 0.0, 0.0);
    if (world_frame) {
      tf2::fromMsg(latest_odom_.pose.pose.orientation, q_body_to_world);
      q_body_to_world.normalize();
      origin = tf2::Vector3(
        latest_odom_.pose.pose.position.x,
        latest_odom_.pose.pose.position.y,
        latest_odom_.pose.pose.position.z);
    }

    const double min_x = get_parameter("crop_min_x").as_double();
    const double max_x = get_parameter("crop_max_x").as_double();
    const double min_y = get_parameter("crop_min_y").as_double();
    const double max_y = get_parameter("crop_max_y").as_double();
    const double min_z = get_parameter("crop_min_z").as_double();
    const double max_z = get_parameter("crop_max_z").as_double();

    CloudT cropped;
    cropped.header = cloud->header;
    cropped.points.reserve(cloud->points.size());
    for (const auto & point : cloud->points) {
      tf2::Vector3 local(point.x, point.y, point.z);
      if (world_frame) {
        const tf2::Vector3 world_delta = local - origin;
        local = tf2::quatRotate(q_body_to_world.inverse(), world_delta);
      }

      if (local.x() >= min_x && local.x() <= max_x &&
        local.y() >= min_y && local.y() <= max_y &&
        local.z() >= min_z && local.z() <= max_z)
      {
        cropped.points.push_back(point);
      }
    }

    cropped.width = static_cast<std::uint32_t>(cropped.points.size());
    cropped.height = 1;
    cropped.is_dense = true;
    *cloud = cropped;
    return true;
  }

  void apply_voxel_filter(CloudT::Ptr cloud) const
  {
    const double leaf = get_parameter("voxel_leaf_size").as_double();
    if (leaf <= 0.0 || cloud->empty()) {
      return;
    }

    pcl::VoxelGrid<PointType> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(
      static_cast<float>(leaf),
      static_cast<float>(leaf),
      static_cast<float>(leaf));
    CloudT filtered;
    voxel.filter(filtered);
    *cloud = filtered;
  }

  void apply_outlier_filter(CloudT::Ptr cloud) const
  {
    const auto type = get_parameter("outlier_filter_type").as_string();
    if (type == "none" || cloud->empty()) {
      return;
    }

    CloudT filtered;
    if (type == "statistical") {
      pcl::StatisticalOutlierRemoval<PointType> sor;
      sor.setInputCloud(cloud);
      sor.setMeanK(std::max(1, static_cast<int>(get_parameter("statistical_mean_k").as_int())));
      sor.setStddevMulThresh(get_parameter("statistical_stddev_mul").as_double());
      sor.filter(filtered);
    } else if (type == "radius") {
      pcl::RadiusOutlierRemoval<PointType> radius;
      radius.setInputCloud(cloud);
      radius.setRadiusSearch(get_parameter("radius_search").as_double());
      radius.setMinNeighborsInRadius(
        std::max(1, static_cast<int>(get_parameter("radius_min_neighbors").as_int())));
      radius.filter(filtered);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unknown outlier_filter_type '%s'. Use none, statistical, or radius.",
        type.c_str());
      return;
    }

    *cloud = filtered;
  }

  void publish_cloud(
    const CloudT & cloud,
    const std_msgs::msg::Header & input_header) const
  {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header = input_header;
    const auto output_frame = get_parameter("output_frame_id").as_string();
    if (!output_frame.empty()) {
      output.header.frame_id = output_frame;
    }
    cloud_pub_->publish(output);
  }

  nav_msgs::msg::Odometry latest_odom_;
  bool has_odom_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudPreprocessor>());
  rclcpp::shutdown();
  return 0;
}
