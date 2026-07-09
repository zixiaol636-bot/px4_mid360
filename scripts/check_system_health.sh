#!/bin/bash
# ============================================================
# System health check before flight.
# Verifies sensors, XRCE-DDS topics, and ROS 2 graph health.
# ============================================================

set -u

echo "=== PX4 + MID360 System Health Check ==="
echo ""

echo "[1/5] Checking MID360 connection..."
if ping -c 3 -W 1 192.168.1.100 > /dev/null 2>&1; then
    echo "  OK: MID360 reachable at 192.168.1.100"
else
    echo "  FAIL: MID360 NOT reachable, check Ethernet connection"
fi

echo "[2/5] Checking PX4 serial connection..."
if [ -e /dev/ttyACM0 ]; then
    echo "  OK: Pixhawk found at /dev/ttyACM0"
elif [ -e /dev/ttyUSB0 ]; then
    echo "  OK: Pixhawk found at /dev/ttyUSB0"
else
    echo "  FAIL: No Pixhawk device found"
fi

echo "[3/5] Checking ROS 2 topics..."
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    # shellcheck source=/dev/null
    source "$HOME/ros2_ws/install/setup.bash"
fi

for topic in /livox/lidar /livox/imu /fmu/out/vehicle_status /px4_native/status; do
    if ros2 topic info "$topic" > /dev/null 2>&1; then
        rate=$(ros2 topic hz "$topic" --window 5 2>/dev/null | tail -1)
        echo "  OK: $topic (rate: ${rate:-N/A})"
    else
        echo "  FAIL: $topic NOT publishing"
    fi
done

echo "[4/5] Checking TF tree..."
if ros2 run tf2_tools view_frames > /dev/null 2>&1; then
    echo "  OK: TF tree available"
else
    echo "  WARN: TF tree check skipped"
fi

echo "[5/5] Checking PX4 DDS state..."
if ros2 topic echo /fmu/out/vehicle_status --once 2>/dev/null | grep -q "arming_state"; then
    echo "  OK: PX4 XRCE-DDS vehicle_status received"
else
    echo "  FAIL: PX4 XRCE-DDS vehicle_status not received"
fi

echo ""
echo "=== Health check complete ==="
