cmake_minimum_required(VERSION 3.8)
project(relocalizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common registration filters)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(icp_relocalizer src/icp_relocalizer.cpp)
target_link_libraries(icp_relocalizer ${PCL_LIBRARIES})
ament_target_dependencies(icp_relocalizer
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros)

install(TARGETS
  icp_relocalizer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
