#!/bin/bash
# ============================================================
# System health check before flight
# Verifies all sensors, connections, and ROS2 topics
# ============================================================

echo "=== PX4 + MID360 System Health Check ==="
echo ""

# 1. Check Livox MID360 connection
echo "[1/5] Checking MID360 connection..."
if ping -c 3 -W 1 192.168.1.100 > /dev/null 2>&1; then
    echo "  ✅ MID360 reachable at 192.168.1.100"
else
    echo "  ❌ MID360 NOT reachable — check Ethernet connection"
fi

# 2. Check PX4 serial connection
echo "[2/5] Checking PX4 connection..."
if [ -e /dev/ttyACM0 ]; then
    echo "  ✅ Pixhawk found at /dev/ttyACM0"
elif [ -e /dev/ttyUSB0 ]; then
    echo "  ✅ Pixhawk found at /dev/ttyUSB0"
else
    echo "  ❌ No Pixhawk device found"
fi

# 3. Check ROS2 topics
echo "[3/5] Checking ROS2 topics..."
source ~/ros2_ws/install/setup.bash
for topic in /livox/lidar /livox/imu /mavros/state; do
    if ros2 topic info $topic > /dev/null 2>&1; then
        rate=$(ros2 topic hz $topic --window 5 2>/dev/null | tail -1)
        echo "  ✅ $topic (rate: ${rate:-N/A})"
    else
        echo "  ❌ $topic NOT publishing"
    fi
done

# 4. Check TF tree
echo "[4/5] Checking TF tree..."
if ros2 run tf2_tools view_frames > /dev/null 2>&1; then
    echo "  ✅ TF tree available"
else
    echo "  ⚠️  TF tree check skipped"
fi

# 5. Check MAVROS armed state
echo "[5/5] Checking PX4 state..."
if ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true"; then
    echo "  ✅ MAVROS2 connected to FCU"
else
    echo "  ❌ MAVROS2 NOT connected to FCU"
fi

echo ""
echo "=== Health check complete ==="
