#!/bin/bash
# ============================================================
# PX4 + MID360 仓库无人机 — 依赖安装脚本
# 目标系统: Ubuntu 22.04 + ROS2 Humble
# ============================================================
set -e

WORKSPACE="${HOME}/ros2_ws"
echo "=== 安装 PX4 + MID360 仓库无人机依赖 ==="

# --- 1. 系统依赖包 ---
echo "[1/6] 安装系统依赖..."
sudo apt-get update
sudo apt-get install -y \
    python3-vcstool python3-rosdep2 python3-colcon-common-extensions \
    libpcl-dev libyaml-cpp-dev libeigen3-dev \
    ros-humble-pcl-conversions ros-humble-pcl-ros \
    ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs

# --- 2. Livox SDK2 ---
echo "[2/6] 编译 Livox SDK2..."
mkdir -p ${WORKSPACE}
cd ${WORKSPACE}
if [ ! -d "Livox-SDK2" ]; then
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
fi
cd Livox-SDK2 && mkdir -p build && cd build
cmake .. && make -j$(nproc) && sudo make install

# --- 3. 拉取所有 ROS2 包 ---
echo "[3/6] 通过 vcstool 拉取 ROS2 包..."
mkdir -p ${WORKSPACE}/src
cd ${WORKSPACE}/src
# 从项目目录复制 dependencies.repos 到工作空间
cp "$(dirname "$0")/../dependencies.repos" .
vcs import < dependencies.repos

# --- 4. 安装 ROS2 包依赖 ---
echo "[4/6] 安装 ROS2 包依赖..."
cd ${WORKSPACE}
rosdep init || true  # ignore if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# --- 5. 编译 livox_ros_driver2 (必须用官方脚本) ---
echo "[5/6] 编译 livox_ros_driver2..."
cd ${WORKSPACE}/src/livox_ros_driver2
./build.sh humble

# --- 6. 编译所有自定义包 ---
echo "[6/6] 用 colcon 编译所有包..."
cd ${WORKSPACE}
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "=== 安装完成! ==="
echo "加载工作空间: source ${WORKSPACE}/install/setup.bash"
