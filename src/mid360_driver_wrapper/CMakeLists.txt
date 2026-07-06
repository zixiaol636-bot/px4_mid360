cmake_minimum_required(VERSION 3.8)
project(mid360_driver_wrapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------------------------------------------------------
# Driver Monitor 节点
# ---------------------------------------------------------------
add_executable(driver_monitor src/driver_monitor.cpp)
ament_target_dependencies(driver_monitor
  rclcpp
  sensor_msgs
  std_msgs
)

# ---------------------------------------------------------------
# 安装
# ---------------------------------------------------------------
install(TARGETS
  driver_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
