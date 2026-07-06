cmake_minimum_required(VERSION 3.8)
project(map_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(map_saver src/map_saver.cpp)
target_link_libraries(map_saver ${PCL_LIBRARIES})
ament_target_dependencies(map_saver
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs)

add_executable(map_loader src/map_loader.cpp)
target_link_libraries(map_loader ${PCL_LIBRARIES})
ament_target_dependencies(map_loader
  pcl_conversions
  rclcpp
  sensor_msgs
  std_srvs)

add_executable(pcd_to_pgm src/pcd_to_pgm.cpp)
target_link_libraries(pcd_to_pgm ${PCL_LIBRARIES})
ament_target_dependencies(pcd_to_pgm rclcpp)

install(TARGETS
  map_saver
  map_loader
  pcd_to_pgm
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
