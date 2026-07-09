#!/usr/bin/env python3

import argparse
import itertools
import math
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import yaml


Grid = List[List[int]]


def load_pgm(image_path: Path) -> Tuple[int, int, int, List[int]]:
    with image_path.open("rb") as handle:
        magic = handle.readline().strip()
        if magic not in {b"P2", b"P5"}:
            raise ValueError(f"Unsupported PGM format: {magic!r}")

        def next_token() -> bytes:
            while True:
                line = handle.readline()
                if not line:
                    return b""
                stripped = line.strip()
                if not stripped or stripped.startswith(b"#"):
                    continue
                return stripped

        dimensions = next_token().split()
        while len(dimensions) < 2:
            dimensions.extend(next_token().split())
        width, height = int(dimensions[0]), int(dimensions[1])

        max_value = int(next_token())
        if max_value <= 0 or max_value > 65535:
            raise ValueError("Invalid PGM max value")

        if magic == b"P5":
            pixel_bytes = handle.read()
            if max_value < 256:
                pixels = list(pixel_bytes[: width * height])
            else:
                pixels = [
                    int.from_bytes(pixel_bytes[index:index + 2], "big")
                    for index in range(0, width * height * 2, 2)
                ]
        else:
            pixels = []
            for line in handle:
                stripped = line.strip()
                if not stripped or stripped.startswith(b"#"):
                    continue
                pixels.extend(int(value) for value in stripped.split())

        return width, height, max_value, pixels[: width * height]


def occupancy_grid_from_map_yaml(map_yaml: Path) -> Tuple[Grid, float, Sequence[float]]:
    with map_yaml.open("r", encoding="utf-8") as handle:
        metadata = yaml.safe_load(handle)

    image_path = (map_yaml.parent / metadata["image"]).resolve()
    width, height, max_value, pixels = load_pgm(image_path)

    negate = int(metadata.get("negate", 0))
    occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
    free_thresh = float(metadata.get("free_thresh", 0.196))

    grid: Grid = []
    index = 0
    for _ in range(height):
        row: List[int] = []
        for _ in range(width):
            value = pixels[index]
            index += 1
            normalized = value / float(max_value)
            occupancy = normalized if negate else 1.0 - normalized
            if occupancy >= occupied_thresh:
                row.append(1)
            elif occupancy <= free_thresh:
                row.append(0)
            else:
                row.append(1)
        grid.append(row)

    return grid, float(metadata["resolution"]), metadata["origin"]


def lane_anchor(warehouse: Dict) -> Tuple[float, float]:
    margin = float(warehouse.get("margin", 0.8))
    length = float(warehouse["length"])
    center = warehouse["center"]
    yaw_rad = math.radians(float(warehouse.get("yaw_deg", 0.0)))
    local_y = -length * 0.5 + margin
    x_world = center[0] - math.sin(yaw_rad) * local_y
    y_world = center[1] + math.cos(yaw_rad) * local_y
    return (x_world, y_world)


def world_to_grid(x_world: float, y_world: float, resolution: float, origin: Sequence[float], height: int) -> Tuple[int, int]:
    grid_x = int(math.floor((x_world - origin[0]) / resolution))
    grid_y = int(math.floor((y_world - origin[1]) / resolution))
    row = height - 1 - grid_y
    col = grid_x
    return row, col


def grid_to_world(row: int, col: int, resolution: float, origin: Sequence[float], height: int) -> Tuple[float, float]:
    grid_y = height - 1 - row
    x_world = origin[0] + (col + 0.5) * resolution
    y_world = origin[1] + (grid_y + 0.5) * resolution
    return x_world, y_world


def nearest_free_cell(grid: Grid, row: int, col: int) -> Tuple[int, int]:
    height = len(grid)
    width = len(grid[0])
    if 0 <= row < height and 0 <= col < width and grid[row][col] == 0:
        return row, col

    queue = deque([(row, col)])
    visited = {(row, col)}
    while queue:
        current_row, current_col = queue.popleft()
        for delta_row, delta_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            next_row = current_row + delta_row
            next_col = current_col + delta_col
            if (next_row, next_col) in visited:
                continue
            visited.add((next_row, next_col))
            if 0 <= next_row < height and 0 <= next_col < width:
                if grid[next_row][next_col] == 0:
                    return next_row, next_col
                queue.append((next_row, next_col))
    raise RuntimeError("Could not find a free cell near the requested point")


def astar_path(grid: Grid, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
    import heapq

    height = len(grid)
    width = len(grid[0])
    moves = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]

    open_heap = [(0.0, 0.0, start)]
    best_g = {start: 0.0}
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

    while open_heap:
        _, g_score, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        current_row, current_col = current
        for delta_row, delta_col, cost in moves:
            next_row = current_row + delta_row
            next_col = current_col + delta_col
            if not (0 <= next_row < height and 0 <= next_col < width):
                continue
            if grid[next_row][next_col] != 0:
                continue
            if delta_row != 0 and delta_col != 0:
                if grid[current_row][next_col] != 0 or grid[next_row][current_col] != 0:
                    continue
            next_cell = (next_row, next_col)
            next_g = g_score + cost
            if next_g >= best_g.get(next_cell, float("inf")):
                continue
            best_g[next_cell] = next_g
            came_from[next_cell] = current
            heuristic = math.hypot(goal[0] - next_row, goal[1] - next_col)
            heapq.heappush(open_heap, (next_g + heuristic, next_g, next_cell))

    return None


def path_cost(path: Optional[Sequence[Tuple[int, int]]]) -> float:
    if not path:
        return float("inf")
    total = 0.0
    for previous, current in zip(path, path[1:]):
        total += math.hypot(current[0] - previous[0], current[1] - previous[1])
    return total


def astar_cost(grid: Grid, start: Tuple[int, int], goal: Tuple[int, int]) -> float:
    path = astar_path(grid, start, goal)
    if path is not None:
        return path_cost(path)
    return float("inf")


def route_targets(layout: Dict) -> List[Tuple[str, Tuple[float, float]]]:
    warehouses = list(layout.get("warehouses", []))
    if not warehouses:
        return []

    return [(warehouse["name"], lane_anchor(warehouse)) for warehouse in warehouses]


def optimize_route(layout: Dict, grid: Grid, resolution: float, origin: Sequence[float]) -> List[str]:
    targets = route_targets(layout)
    if not targets:
        return []

    start_xy = layout["start_pose"]["position"][:2]
    height = len(grid)

    def anchor_cell(xy: Sequence[float]) -> Tuple[int, int]:
        row, col = world_to_grid(xy[0], xy[1], resolution, origin, height)
        return nearest_free_cell(grid, row, col)

    start_cell = anchor_cell(start_xy)
    warehouse_cells = {name: anchor_cell(xy) for name, xy in targets}
    warehouses = [warehouse for warehouse in layout.get("warehouses", [])]
    cost_cache: Dict[Tuple[Tuple[int, int], Tuple[int, int]], float] = {}

    def cached_cost(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        key = (a, b)
        if key not in cost_cache:
            cost_cache[key] = astar_cost(grid, a, b)
        return cost_cache[key]

    def order_cost(order: Sequence[Dict]) -> float:
        total_cost = 0.0
        previous_cell = start_cell
        for warehouse in order:
            current_cell = warehouse_cells[warehouse["name"]]
            total_cost += cached_cost(previous_cell, current_cell)
            previous_cell = current_cell
        if layout.get("return_home", True):
            total_cost += cached_cost(previous_cell, start_cell)
        return total_cost * resolution

    best_order = warehouses
    best_cost = float("inf")
    for permutation in itertools.permutations(warehouses):
        current_cost = order_cost(permutation)
        if current_cost < best_cost:
            best_cost = current_cost
            best_order = list(permutation)

    print(f"Optimized route length estimate: {best_cost:.2f} m")
    return [warehouse["name"] for warehouse in best_order]


def yaw_to_quaternion(yaw_rad: float) -> List[float]:
    half_yaw = yaw_rad * 0.5
    return [0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)]


def simplify_path(path: Sequence[Tuple[int, int]], step: int) -> List[Tuple[int, int]]:
    if len(path) <= 2:
        return list(path)
    step = max(1, step)
    simplified = [path[0]]
    simplified.extend(path[index] for index in range(step, len(path) - 1, step))
    if simplified[-1] != path[-1]:
        simplified.append(path[-1])
    return simplified


def build_global_astar_waypoints(
    layout: Dict,
    grid: Grid,
    resolution: float,
    origin: Sequence[float],
    visit_order: Sequence[str],
    altitude: float,
    hover_time: float,
    acceptance_radius: float,
    simplify_step: int,
) -> List[Dict]:
    height = len(grid)

    def anchor_cell(xy: Sequence[float]) -> Tuple[int, int]:
        row, col = world_to_grid(xy[0], xy[1], resolution, origin, height)
        return nearest_free_cell(grid, row, col)

    start_xy = layout["start_pose"]["position"][:2]
    start_cell = anchor_cell(start_xy)
    target_lookup = {name: xy for name, xy in route_targets(layout)}
    ordered_targets = [target_lookup[name] for name in visit_order if name in target_lookup]
    target_cells = [anchor_cell(xy) for xy in ordered_targets]
    if layout.get("return_home", True):
        target_cells.append(start_cell)

    cells: List[Tuple[int, int]] = []
    previous = start_cell
    for target in target_cells:
        segment = astar_path(grid, previous, target)
        if segment is None:
            raise RuntimeError(f"No A* path from {previous} to {target}")
        if cells:
            cells.extend(segment[1:])
        else:
            cells.extend(segment)
        previous = target

    cells = simplify_path(cells, simplify_step)
    waypoints: List[Dict] = []
    previous_xy: Optional[Tuple[float, float]] = None
    for row, col in cells:
        x_world, y_world = grid_to_world(row, col, resolution, origin, height)
        if previous_xy is None:
            yaw = math.radians(float(layout.get("start_pose", {}).get("yaw_deg", 0.0)))
        else:
            dx = x_world - previous_xy[0]
            dy = y_world - previous_xy[1]
            yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else 0.0
        waypoints.append({
            "frame_id": layout.get("frame_id", "map"),
            "position": [round(x_world, 3), round(y_world, 3), round(altitude, 3)],
            "orientation": [round(value, 6) for value in yaw_to_quaternion(yaw)],
            "hover_time": hover_time,
            "acceptance_radius": acceptance_radius,
            "source": "astar_global",
        })
        previous_xy = (x_world, y_world)

    return waypoints


def main() -> None:
    parser = argparse.ArgumentParser(description="Optimize six-warehouse visit order using occupancy-grid A*.")
    parser.add_argument("layout_file", help="Input warehouse layout YAML file")
    parser.add_argument("map_yaml", help="Occupancy grid map YAML file used for A* search")
    parser.add_argument(
        "--output-layout",
        help="Optional path to write a copy of the layout with visit_order updated",
    )
    parser.add_argument(
        "--output-waypoints",
        help="Optional path to write A* global path waypoints for mission_executor",
    )
    parser.add_argument("--altitude", type=float, default=None, help="A* waypoint altitude. Defaults to start_pose z.")
    parser.add_argument("--hover-time", type=float, default=0.0, help="Hover time for generated A* waypoints.")
    parser.add_argument(
        "--acceptance-radius",
        type=float,
        default=0.8,
        help="Acceptance radius for generated A* waypoints.",
    )
    parser.add_argument(
        "--path-simplify-step",
        type=int,
        default=5,
        help="Keep every Nth A* cell when writing waypoint YAML.",
    )
    args = parser.parse_args()

    layout_path = Path(args.layout_file)
    with layout_path.open("r", encoding="utf-8") as handle:
        layout = yaml.safe_load(handle) or {}

    grid, resolution, origin = occupancy_grid_from_map_yaml(Path(args.map_yaml))
    best_order = optimize_route(layout, grid, resolution, origin)

    print("Recommended visit order:")
    for index, name in enumerate(best_order, start=1):
        print(f"  {index}. {name}")

    if args.output_layout:
        layout["visit_order"] = best_order
        output_path = Path(args.output_layout)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(layout, handle, sort_keys=False, allow_unicode=False)
        print(f"Wrote optimized layout -> {output_path}")

    if args.output_waypoints:
        altitude = args.altitude
        if altitude is None:
            altitude = float(layout.get("start_pose", {}).get("position", [0.0, 0.0, 1.0])[2])
        waypoints = build_global_astar_waypoints(
            layout,
            grid,
            resolution,
            origin,
            best_order,
            altitude,
            args.hover_time,
            args.acceptance_radius,
            args.path_simplify_step,
        )
        output_path = Path(args.output_waypoints)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(waypoints, handle, sort_keys=False, allow_unicode=False)
        print(f"Wrote A* global waypoints -> {output_path}")


if __name__ == "__main__":
    main()
