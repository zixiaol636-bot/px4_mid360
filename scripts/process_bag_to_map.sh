#!/bin/bash
# ============================================================
# End-to-end: ros2 bag → PCD map → PGM grid
# ============================================================
set -e

BAG_PATH="${1:-./bags/warehouse_mapping}"
MAP_NAME="${2:-warehouse_map}"
MAP_DIR="./maps"

echo "=== Processing bag: ${BAG_PATH} ==="

# 1. Play bag into FAST-LIO2 to build map
echo "[1/3] Building point cloud map from bag..."
ros2 launch px4_mid360 build_map_offline.launch.py bag_path:=${BAG_PATH}
sleep 3

# 2. Save PCD map via service
echo "[2/3] Saving PCD map..."
ros2 service call /map/save map_manager_interfaces/srv/SaveMap \
    "{map_path: '${MAP_DIR}/${MAP_NAME}.pcd'}"
sleep 2

# 3. Convert PCD to Nav2-compatible PGM/YAML
echo "[3/3] Converting PCD to PGM/YAML..."
ros2 run map_manager pcd_to_pgm --ros-args \
    -p pcd_path:=${MAP_DIR}/${MAP_NAME}.pcd \
    -p resolution:=0.1 \
    -p z_min:=-0.5 -p z_max:=3.5

echo "=== Done! Output: ${MAP_DIR}/${MAP_NAME}.pcd, .pgm, .yaml ==="
