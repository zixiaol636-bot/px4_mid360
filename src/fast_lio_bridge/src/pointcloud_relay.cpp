#include <memory>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointcloudRelay : public rclcpp::Node
{
public:
  PointcloudRelay() : Node("pointcloud_relay")
  {
    this->declare_parameter<std::string>("cloud_topic_in", "/cloud_registered");
    this->declare_parameter<std::string>("cloud_topic_out", "/cloud_registered_relay");
    this->declare_parameter<std::string>("output_frame_id", "");
    this->declare_parameter<double>("voxel_leaf_size", 0.0);

    const auto topic_in = this->get_parameter("cloud_topic_in").as_string();
    const auto topic_out = this->get_parameter("cloud_topic_out").as_string();

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_in, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud_callback(msg); });

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      topic_out, rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "PointcloudRelay: %s -> %s", topic_in.c_str(), topic_out.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto output_frame = this->get_parameter("output_frame_id").as_string();
    const double voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();

    sensor_msgs::msg::PointCloud2 out_msg;
    if (voxel_leaf_size > 0.0) {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::fromROSMsg(*msg, cloud);

      pcl::VoxelGrid<pcl::PointXYZI> voxel;
      voxel.setInputCloud(cloud.makeShared());
      voxel.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
      pcl::PointCloud<pcl::PointXYZI> filtered;
      voxel.filter(filtered);
      pcl::toROSMsg(filtered, out_msg);
      out_msg.header = msg->header;
    } else {
      out_msg = *msg;
    }

    if (!output_frame.empty()) {
      out_msg.header.frame_id = output_frame;
    }
    cloud_pub_->publish(out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudRelay>());
  rclcpp::shutdown();
  return 0;
}
