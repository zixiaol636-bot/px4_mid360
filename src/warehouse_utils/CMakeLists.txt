cmake_minimum_required(VERSION 3.8)  # CMake 最低版本要求，ROS2 Humble 标配 >= 3.8
project(warehouse_utils)             # 项目名，需与 package.xml 中的 <name> 一致

# ---- C++ 标准 ----
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---- 查找依赖 ----
find_package(ament_cmake REQUIRED)        # ROS2 构建系统
find_package(rclcpp REQUIRED)             # ROS2 C++ 客户端库
find_package(tf2 REQUIRED)                # TF2 坐标变换核心
find_package(tf2_geometry_msgs REQUIRED)  # tf2 与 geometry_msgs 互转
find_package(geometry_msgs REQUIRED)      # Pose / Twist / Point 等消息类型
find_package(yaml-cpp REQUIRED)           # YAML 文件读写

# ===== 库1: frame_conversions =====
# 提供 FLU ↔ NED 坐标系转换（5 种重载 + yaw_rate）
add_library(frame_conversions SHARED
  src/frame_conversions.cpp)

# 让其他包 #include "warehouse_utils/frame_conversions.hpp" 能找到头文件
target_include_directories(frame_conversions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ament_target_dependencies 自动处理头文件路径 + 链接库，替代 target_link_libraries
ament_target_dependencies(frame_conversions
  geometry_msgs
  tf2
  tf2_geometry_msgs)

# ===== 库2: yaml_utils =====
# 提供航点 YAML 读写 + ROS2 参数提取
add_library(yaml_utils SHARED
  src/yaml_utils.cpp)

target_include_directories(yaml_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# yaml-cpp 不是 ament 包，需手动 link
target_link_libraries(yaml_utils yaml-cpp)
ament_target_dependencies(yaml_utils
  geometry_msgs
  rclcpp)

# ---- 安装 ----
# 把编译产物安装到 install/ 目录，其他包通过 ament 找到它们
install(TARGETS
  frame_conversions
  yaml_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib     # .a 静态库
  LIBRARY DESTINATION lib     # .so 动态库
  RUNTIME DESTINATION bin)    # .exe 可执行文件

install(DIRECTORY include/
  DESTINATION include)

# ---- 导出 ----
# 让依赖本包的其他包能自动找到头文件、库文件和依赖
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(frame_conversions yaml_utils)
ament_export_dependencies(
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs)
ament_package()
