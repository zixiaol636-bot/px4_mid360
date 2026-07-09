#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

using PointType = pcl::PointXYZI;

class PcdToPgm : public rclcpp::Node
{
public:
  PcdToPgm() : Node("pcd_to_pgm")
  {
    this->declare_parameter<double>("resolution", 0.1);
    this->declare_parameter<double>("z_min", -0.5);
    this->declare_parameter<double>("z_max", 3.5);
    this->declare_parameter<std::string>("input_pcd", "/tmp/map.pcd");
    this->declare_parameter<std::string>("output_dir", "/tmp");
    this->declare_parameter<bool>("auto_run", true);

    if (this->get_parameter("auto_run").as_bool()) {
      run_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() {
          run_timer_->cancel();
          convert();
          rclcpp::shutdown();
        });
    }
  }

private:
  void convert()
  {
    const auto input_pcd = this->get_parameter("input_pcd").as_string();
    const auto output_dir = this->get_parameter("output_dir").as_string();
    const double resolution = this->get_parameter("resolution").as_double();
    const double z_min = this->get_parameter("z_min").as_double();
    const double z_max = this->get_parameter("z_max").as_double();
    std::filesystem::create_directories(output_dir);

    pcl::PointCloud<PointType> cloud;
    if (pcl::io::loadPCDFile(input_pcd, cloud) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load input PCD: %s", input_pcd.c_str());
      return;
    }

    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pcl::PointCloud<PointType> cropped;
    pass.filter(cropped);
    if (cropped.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No points remain after z filtering.");
      return;
    }

    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();
    for (const auto & point : cropped.points) {
      x_min = std::min<double>(x_min, point.x);
      x_max = std::max<double>(x_max, point.x);
      y_min = std::min<double>(y_min, point.y);
      y_max = std::max<double>(y_max, point.y);
    }

    const int width = static_cast<int>(std::ceil((x_max - x_min) / resolution)) + 1;
    const int height = static_cast<int>(std::ceil((y_max - y_min) / resolution)) + 1;
    std::vector<std::vector<bool>> grid(height, std::vector<bool>(width, false));

    for (const auto & point : cropped.points) {
      const int col = static_cast<int>(std::floor((point.x - x_min) / resolution));
      const int row = static_cast<int>(std::floor((point.y - y_min) / resolution));
      if (row >= 0 && row < height && col >= 0 && col < width) {
        grid[row][col] = true;
      }
    }

    const auto pgm_path = output_dir + "/map.pgm";
    std::ofstream pgm(pgm_path, std::ios::binary);
    pgm << "P5\n" << width << " " << height << "\n255\n";
    for (int row = height - 1; row >= 0; --row) {
      for (int col = 0; col < width; ++col) {
        pgm.put(static_cast<char>(grid[row][col] ? 0 : 254));
      }
    }

    const auto yaml_path = output_dir + "/map.yaml";
    std::ofstream yaml(yaml_path);
    yaml << "image: map.pgm\n";
    yaml << "mode: trinary\n";
    yaml << "resolution: " << resolution << "\n";
    yaml << "origin: [" << x_min << ", " << y_min << ", 0.0]\n";
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.25\n";
  }

  rclcpp::TimerBase::SharedPtr run_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdToPgm>());
  rclcpp::shutdown();
  return 0;
}
