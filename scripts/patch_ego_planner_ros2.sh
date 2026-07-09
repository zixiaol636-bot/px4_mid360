#!/bin/bash
# Patch ego-planner-swarm ros2_version for real waypoint altitude support.

set -euo pipefail

WORKSPACE="${WORKSPACE:-${HOME}/ros2_ws}"
EGO_ROOT="${EGO_ROOT:-${WORKSPACE}/src/ego_planner_swarm_ros2}"
TARGET_FILE="${EGO_ROOT}/src/planner/plan_manage/src/ego_replan_fsm.cpp"

if [ ! -f "${TARGET_FILE}" ]; then
    echo "EGO-Planner source not found: ${TARGET_FILE}" >&2
    echo "Run vcs import first, or set EGO_ROOT=/path/to/ego_planner_swarm_ros2" >&2
    exit 1
fi

OLD_LINE='Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);'
NEW_LINE='Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);'

if grep -Fq "${NEW_LINE}" "${TARGET_FILE}"; then
    echo "EGO-Planner altitude patch already applied."
    exit 0
fi

if ! grep -Fq "${OLD_LINE}" "${TARGET_FILE}"; then
    echo "Could not find the expected fixed-altitude line in ${TARGET_FILE}" >&2
    echo "Inspect waypointCallback() manually before real flight." >&2
    exit 1
fi

cp "${TARGET_FILE}" "${TARGET_FILE}.bak"
sed -i "s|${OLD_LINE}|${NEW_LINE}|" "${TARGET_FILE}"
echo "Patched EGO-Planner goal z altitude: ${TARGET_FILE}"
