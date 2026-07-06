#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import yaml


def yaw_to_quaternion(yaw_rad: float) -> List[float]:
    half_yaw = yaw_rad * 0.5
    return [0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)]


def world_from_local(center: Sequence[float], yaw_rad: float, x_local: float, y_local: float) -> Tuple[float, float]:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    x_world = center[0] + cos_yaw * x_local - sin_yaw * y_local
    y_world = center[1] + sin_yaw * x_local + cos_yaw * y_local
    return x_world, y_world


def lane_positions(width: float, margin: float, lane_spacing: float) -> List[float]:
    usable_half_width = max(width * 0.5 - margin, 0.0)
    if usable_half_width <= 1e-6:
        return [0.0]

    positions: List[float] = []
    x_value = -usable_half_width
    while x_value <= usable_half_width + 1e-6:
        positions.append(round(x_value, 3))
        x_value += max(lane_spacing, 0.5)

    if positions[-1] < usable_half_width - 1e-3:
        positions.append(round(usable_half_width, 3))

    return positions


def build_local_pattern(warehouse: Dict) -> List[Tuple[float, float]]:
    length = float(warehouse["length"])
    width = float(warehouse["width"])
    margin = float(warehouse.get("margin", 0.8))
    lane_spacing = float(warehouse.get("lane_spacing", 1.8))

    near_y = -length * 0.5 + margin
    far_y = length * 0.5 - margin
    entry = (0.0, near_y)
    lanes = lane_positions(width, margin, lane_spacing)

    pattern: List[Tuple[float, float]] = [entry]
    if not lanes:
        return pattern

    current = entry
    for index, lane_x in enumerate(lanes):
        lane_start = (lane_x, near_y if index % 2 == 0 else far_y)
        if current != lane_start:
            pattern.append(lane_start)

        lane_end = (lane_x, far_y if index % 2 == 0 else near_y)
        pattern.append(lane_end)
        current = lane_end

        if index != len(lanes) - 1:
            next_lane_x = lanes[index + 1]
            pattern.append((next_lane_x, lane_end[1]))
            current = pattern[-1]

    if pattern[-1] != entry:
        pattern.append(entry)

    return pattern


def order_warehouses(layout: Dict) -> List[Dict]:
    warehouses = list(layout.get("warehouses", []))
    explicit_order = layout.get("visit_order")
    if not explicit_order:
        return warehouses

    lookup = {warehouse["name"]: warehouse for warehouse in warehouses}
    ordered = []
    for name in explicit_order:
        if name in lookup:
            ordered.append(lookup[name])
    remaining = [warehouse for warehouse in warehouses if warehouse["name"] not in set(explicit_order)]
    return ordered + remaining


def build_waypoints(layout: Dict) -> List[Dict]:
    frame_id = layout.get("frame_id", "map")
    default_hover = float(layout.get("default_hover_time", 1.5))
    default_acceptance = float(layout.get("default_acceptance_radius", 0.8))
    mission_waypoints: List[Dict] = []

    for warehouse in order_warehouses(layout):
        center = warehouse["center"]
        altitude = float(warehouse.get("altitude", layout["start_pose"]["position"][2]))
        yaw_rad = math.radians(float(warehouse.get("yaw_deg", 0.0)))
        hover_time = float(warehouse.get("hover_time", default_hover))
        acceptance = float(warehouse.get("acceptance_radius", default_acceptance))

        local_pattern = build_local_pattern(warehouse)
        previous_world = None
        for x_local, y_local in local_pattern:
            x_world, y_world = world_from_local(center, yaw_rad, x_local, y_local)
            if previous_world is None:
                waypoint_yaw = yaw_rad
            else:
                dx = x_world - previous_world[0]
                dy = y_world - previous_world[1]
                waypoint_yaw = yaw_rad if math.hypot(dx, dy) < 1e-6 else math.atan2(dy, dx)

            mission_waypoints.append(
                {
                    "frame_id": frame_id,
                    "position": [round(x_world, 3), round(y_world, 3), round(altitude, 3)],
                    "orientation": [round(value, 6) for value in yaw_to_quaternion(waypoint_yaw)],
                    "hover_time": hover_time,
                    "acceptance_radius": acceptance,
                    "warehouse": warehouse["name"],
                }
            )
            previous_world = (x_world, y_world)

    return mission_waypoints


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate six-warehouse inventory waypoints.")
    parser.add_argument("layout_file", help="Input warehouse layout YAML file")
    parser.add_argument("output_file", help="Output waypoint YAML file")
    args = parser.parse_args()

    layout_path = Path(args.layout_file)
    output_path = Path(args.output_file)

    with layout_path.open("r", encoding="utf-8") as handle:
        layout = yaml.safe_load(handle) or {}

    mission_waypoints = build_waypoints(layout)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(mission_waypoints, handle, sort_keys=False, allow_unicode=False)

    print(f"Generated {len(mission_waypoints)} waypoints -> {output_path}")


if __name__ == "__main__":
    main()
