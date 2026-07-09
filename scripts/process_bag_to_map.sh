#!/bin/bash
# Replay a mapping bag, save a PCD map, and convert it to an occupancy grid.

set -euo pipefail

BAG_PATH="${1:-./bags/warehouse_mapping}"
MAP_NAME="${2:-warehouse_map}"
BUILD_SECONDS="${3:-60}"
MAP_DIR="${MAP_DIR:-./maps}"
MAP_PATH="${MAP_DIR}/${MAP_NAME}.pcd"

mkdir -p "${MAP_DIR}"

echo "=== Processing bag: ${BAG_PATH} ==="
echo "Map path: ${MAP_PATH}"
echo "Build window: ${BUILD_SECONDS}s"

ros2 launch px4_mid360 build_map_offline.launch.py \
    bag_path:="${BAG_PATH}" \
    map_path:="${MAP_PATH}" \
    save_directory:="${MAP_DIR}" &
LAUNCH_PID=$!

cleanup() {
    if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        kill "${LAUNCH_PID}" 2>/dev/null || true
        wait "${LAUNCH_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT

echo "[1/3] Waiting for FAST-LIO/map_saver to accumulate map..."
sleep "${BUILD_SECONDS}"

echo "[2/3] Triggering /map/save..."
ros2 service call /map/save std_srvs/srv/Trigger "{}"
sleep 2

echo "[3/3] Converting PCD to PGM/YAML..."
ros2 run map_manager pcd_to_pgm --ros-args \
    -p input_pcd:="${MAP_PATH}" \
    -p output_dir:="${MAP_DIR}" \
    -p resolution:=0.1 \
    -p z_min:=-0.5 \
    -p z_max:=3.5 \
    -p auto_run:=true

echo "=== Done ==="
echo "PCD: ${MAP_PATH}"
echo "Occupancy grid map: ${MAP_DIR}/map.yaml"
