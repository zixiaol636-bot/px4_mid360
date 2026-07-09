#!/bin/bash
# Install system packages and source dependencies for the PX4 + MID360 workspace.

set -euo pipefail

WORKSPACE="${WORKSPACE:-${HOME}/ros2_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=== Installing PX4 + MID360 dependencies ==="
echo "Workspace: ${WORKSPACE}"

echo "[1/6] Installing system packages..."
sudo apt-get update
sudo apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep2 \
    python3-vcstool \
    libeigen3-dev \
    libnlopt-dev \
    libopencv-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs

echo "[2/6] Building and installing Livox SDK2..."
mkdir -p "${WORKSPACE}"
cd "${WORKSPACE}"
if [ ! -d "Livox-SDK2" ]; then
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
fi
cmake -S Livox-SDK2 -B Livox-SDK2/build
cmake --build Livox-SDK2/build -j"$(nproc)"
sudo cmake --install Livox-SDK2/build

echo "[3/6] Importing ROS 2 source dependencies..."
mkdir -p "${WORKSPACE}/src"
cd "${WORKSPACE}/src"
if [ ! -f "${REPO_ROOT}/dependencies.repos" ]; then
    echo "Cannot find dependencies.repos at ${REPO_ROOT}/dependencies.repos" >&2
    exit 1
fi
vcs import < "${REPO_ROOT}/dependencies.repos"

echo "[3.1/6] Patching EGO-Planner ROS2 goal altitude handling..."
WORKSPACE="${WORKSPACE}" bash "${REPO_ROOT}/scripts/patch_ego_planner_ros2.sh"

echo "[4/6] Installing ROS package dependencies..."
cd "${WORKSPACE}"
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "Eigen3 PCL OpenCV"

echo "[5/6] Building livox_ros_driver2 with its official script..."
cd "${WORKSPACE}/src/livox_ros_driver2"
./build.sh humble

echo "[6/6] Building workspace..."
cd "${WORKSPACE}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "=== Installation complete ==="
echo "Run: source ${WORKSPACE}/install/setup.bash"
