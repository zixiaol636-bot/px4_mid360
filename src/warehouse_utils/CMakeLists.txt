cmake_minimum_required(VERSION 3.8)
project(warehouse_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(frame_conversions SHARED
  src/frame_conversions.cpp
)
target_include_directories(frame_conversions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(frame_conversions
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_library(yaml_utils SHARED
  src/yaml_utils.cpp
)
target_include_directories(yaml_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(yaml_utils yaml-cpp)
ament_target_dependencies(yaml_utils
  geometry_msgs
  rclcpp
)

install(TARGETS
  frame_conversions
  yaml_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(frame_conversions yaml_utils)
ament_export_dependencies(
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

ament_package()
