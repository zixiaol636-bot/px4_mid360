#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using PointType = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointType>;

class IcpRelocalizer : public rclcpp::Node
{
public:
  IcpRelocalizer() : Node("icp_relocalizer"), tf_broadcaster_(this)
  {
    this->declare_parameter<double>("icp_max_corr_dist", 1.0);
    this->declare_parameter<int>("icp_max_iter", 50);
    this->declare_parameter<double>("icp_fitness_thresh", 0.15);
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<double>("voxel_leaf_size", 0.2);
    this->declare_parameter<double>("correction_rate", 0.5);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { latest_scan_ = *msg; has_scan_ = true; });

    map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/map_cloud", rclcpp::QoS(1).transient_local().reliable(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { latest_map_ = *msg; has_map_ = true; });

    trigger_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/relocalize/trigger",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        response->success = run_icp_once();
        response->message = response->success ? "Relocalization succeeded." : "Relocalization failed.";
      });

    const double correction_rate = this->get_parameter("correction_rate").as_double();
    correction_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / correction_rate),
      [this]() {
        if (has_transform_) {
          run_icp_once();
        }
      });
    correction_timer_->cancel();
  }

private:
  bool run_icp_once()
  {
    if (!has_map_ || !has_scan_) {
      return false;
    }

    CloudT::Ptr map_cloud(new CloudT());
    CloudT::Ptr scan_cloud(new CloudT());
    pcl::fromROSMsg(latest_map_, *map_cloud);
    pcl::fromROSMsg(latest_scan_, *scan_cloud);
    if (map_cloud->empty() || scan_cloud->empty()) {
      return false;
    }

    const double voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
    if (voxel_leaf_size > 0.0) {
      downsample(scan_cloud, voxel_leaf_size);
      downsample(map_cloud, voxel_leaf_size);
    }

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(scan_cloud);
    icp.setInputTarget(map_cloud);
    icp.setMaxCorrespondenceDistance(this->get_parameter("icp_max_corr_dist").as_double());
    icp.setMaximumIterations(this->get_parameter("icp_max_iter").as_int());
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-3);

    CloudT aligned;
    const Eigen::Matrix4f initial_guess = has_transform_ ? transform_map_odom_ : Eigen::Matrix4f::Identity();
    icp.align(aligned, initial_guess);
    if (!icp.hasConverged() ||
        icp.getFitnessScore() > this->get_parameter("icp_fitness_thresh").as_double())
    {
      return false;
    }

    transform_map_odom_ = icp.getFinalTransformation();
    has_transform_ = true;
    correction_timer_->reset();
    publish_transform();
    return true;
  }

  void downsample(CloudT::Ptr cloud, double voxel_leaf_size)
  {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    CloudT filtered;
    voxel.filter(filtered);
    *cloud = filtered;
  }

  void publish_transform()
  {
    const auto map_frame = this->get_parameter("map_frame").as_string();
    const auto odom_frame = this->get_parameter("odom_frame").as_string();
    const Eigen::Matrix3f rotation = transform_map_odom_.block<3, 3>(0, 0);
    const Eigen::Vector3f translation = transform_map_odom_.block<3, 1>(0, 3);
    const Eigen::Quaternionf quaternion(rotation);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = map_frame;
    tf_msg.child_frame_id = odom_frame;
    tf_msg.transform.translation.x = translation.x();
    tf_msg.transform.translation.y = translation.y();
    tf_msg.transform.translation.z = translation.z();
    tf_msg.transform.rotation.x = quaternion.x();
    tf_msg.transform.rotation.y = quaternion.y();
    tf_msg.transform.rotation.z = quaternion.z();
    tf_msg.transform.rotation.w = quaternion.w();
    tf_broadcaster_.sendTransform(tf_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  rclcpp::TimerBase::SharedPtr correction_timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  sensor_msgs::msg::PointCloud2 latest_scan_;
  sensor_msgs::msg::PointCloud2 latest_map_;
  bool has_scan_{false};
  bool has_map_{false};
  bool has_transform_{false};
  Eigen::Matrix4f transform_map_odom_{Eigen::Matrix4f::Identity()};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IcpRelocalizer>());
  rclcpp::shutdown();
  return 0;
}
