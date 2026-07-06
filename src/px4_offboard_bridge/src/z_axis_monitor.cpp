#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

class ZAxisMonitor : public rclcpp::Node
{
public:
  ZAxisMonitor() : Node("z_axis_monitor")
  {
    this->declare_parameter<std::string>("pointcloud_topic", "/cloud_registered");
    this->declare_parameter<std::string>("obstacle_z_topic", "/obstacle/z_status");
    this->declare_parameter<std::vector<double>>("check_range", {-2.0, 2.0});
    this->declare_parameter<int>("point_density_threshold", 15);
    this->declare_parameter<double>("min_z_response", -0.3);
    this->declare_parameter<double>("max_z_response", 0.3);
    this->declare_parameter<double>("clear_timeout", 1.0);
    this->declare_parameter<double>("publish_rate", 5.0);

    const auto pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
    const auto obstacle_z_topic = this->get_parameter("obstacle_z_topic").as_string();
    const double publish_rate = this->get_parameter("publish_rate").as_double();

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { latest_cloud_ = msg; });

    obstacle_pub_ = this->create_publisher<std_msgs::msg::Float32>(obstacle_z_topic, rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      [this]() { evaluate_latest_cloud(); });
  }

private:
  void evaluate_latest_cloud()
  {
    if (!latest_cloud_) {
      publish_override(0.0);
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*latest_cloud_, cloud);
    if (cloud.empty()) {
      publish_override(0.0);
      return;
    }

    const auto check_range = this->get_parameter("check_range").as_double_array();
    const int density_threshold = this->get_parameter("point_density_threshold").as_int();
    const double min_z_response = this->get_parameter("min_z_response").as_double();
    const double max_z_response = this->get_parameter("max_z_response").as_double();

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(check_range[0], check_range[1]);

    pcl::PointCloud<pcl::PointXYZI> filtered;
    pass.filter(filtered);

    if (static_cast<int>(filtered.size()) < density_threshold) {
      publish_override(0.0);
      return;
    }

    double average_z = 0.0;
    for (const auto & point : filtered.points) {
      average_z += point.z;
    }
    average_z /= static_cast<double>(filtered.size());

    // ROS FLU convention: positive z is up. Negative average_z means obstacle below.
    const double z_override = average_z > 0.0 ? min_z_response : max_z_response;
    publish_override(z_override);
  }

  void publish_override(double value)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(value);
    obstacle_pub_->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr obstacle_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZAxisMonitor>());
  rclcpp::shutdown();
  return 0;
}
