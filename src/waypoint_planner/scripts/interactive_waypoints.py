#!/usr/bin/env python3

import os
from typing import List

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from waypoint_planner.msg import Waypoint
from waypoint_planner.srv import LoadWaypoints, SaveWaypoints


class InteractiveWaypointTool(Node):
    def __init__(self) -> None:
        super().__init__("interactive_waypoints")

        self.declare_parameter("waypoints_file", "waypoints.yaml")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("hover_time", 3.0)
        self.declare_parameter("acceptance_radius", 1.0)
        self.declare_parameter("auto_load_on_start", True)
        self.declare_parameter("save_on_shutdown", True)

        self.waypoints_file = self.get_parameter("waypoints_file").value
        self.map_frame = self.get_parameter("map_frame").value
        self.hover_time = float(self.get_parameter("hover_time").value)
        self.acceptance_radius = float(self.get_parameter("acceptance_radius").value)

        self.waypoints: List[Waypoint] = []

        self.clicked_sub = self.create_subscription(
            PointStamped, "/clicked_point", self._clicked_point_cb, 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/waypoint_markers", 10
        )
        self.save_client = self.create_client(SaveWaypoints, "/waypoint/save")
        self.load_client = self.create_client(LoadWaypoints, "/waypoint/load")

        if bool(self.get_parameter("auto_load_on_start").value):
            self._load_waypoints()

        self.get_logger().info(
            "Click points in RViz using 'Publish Point' on /clicked_point to add waypoints."
        )

    def _clicked_point_cb(self, msg: PointStamped) -> None:
        waypoint = Waypoint()
        waypoint.pose.position.x = msg.point.x
        waypoint.pose.position.y = msg.point.y
        waypoint.pose.position.z = msg.point.z
        waypoint.pose.orientation.w = 1.0
        waypoint.hover_time = self.hover_time
        waypoint.acceptance_radius = self.acceptance_radius
        self.waypoints.append(waypoint)
        self._publish_markers()
        self.get_logger().info(
            f"Added waypoint #{len(self.waypoints)} at "
            f"({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})"
        )

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        for index, waypoint in enumerate(self.waypoints):
            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = "waypoints"
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = waypoint.pose
            sphere.scale.x = 0.4
            sphere.scale.y = 0.4
            sphere.scale.z = 0.4
            sphere.color.g = 1.0
            sphere.color.a = 0.9
            marker_array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = sphere.header.stamp
            label.ns = "waypoint_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = waypoint.pose.position.x
            label.pose.position.y = waypoint.pose.position.y
            label.pose.position.z = waypoint.pose.position.z + 0.8
            label.scale.z = 0.5
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"WP{index + 1}"
            marker_array.markers.append(label)

        self.marker_pub.publish(marker_array)

    def _save_waypoints(self) -> None:
        if self.save_client.wait_for_service(timeout_sec=0.5):
            request = SaveWaypoints.Request()
            request.waypoints.waypoints = self.waypoints
            future = self.save_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result and result.success:
                self.get_logger().info(result.message)
                return

        payload = []
        for waypoint in self.waypoints:
            payload.append(
                {
                    "frame_id": self.map_frame,
                    "position": [
                        waypoint.pose.position.x,
                        waypoint.pose.position.y,
                        waypoint.pose.position.z,
                    ],
                    "orientation": [
                        waypoint.pose.orientation.x,
                        waypoint.pose.orientation.y,
                        waypoint.pose.orientation.z,
                        waypoint.pose.orientation.w,
                    ],
                    "hover_time": waypoint.hover_time,
                    "acceptance_radius": waypoint.acceptance_radius,
                }
            )
        with open(self.waypoints_file, "w", encoding="utf-8") as handle:
            yaml.safe_dump(payload, handle, sort_keys=False)
        self.get_logger().info(f"Saved {len(self.waypoints)} waypoints to {self.waypoints_file}")

    def _load_waypoints(self) -> None:
        if self.load_client.wait_for_service(timeout_sec=0.5):
            request = LoadWaypoints.Request()
            future = self.load_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result and result.success:
                self.waypoints = list(result.waypoints.waypoints)
                self._publish_markers()
                self.get_logger().info(result.message)
                return

        if not os.path.exists(self.waypoints_file):
            return

        with open(self.waypoints_file, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or []

        self.waypoints = []
        for item in data:
            waypoint = Waypoint()
            waypoint.pose.position.x = item["position"][0]
            waypoint.pose.position.y = item["position"][1]
            waypoint.pose.position.z = item["position"][2]
            waypoint.pose.orientation.x = item["orientation"][0]
            waypoint.pose.orientation.y = item["orientation"][1]
            waypoint.pose.orientation.z = item["orientation"][2]
            waypoint.pose.orientation.w = item["orientation"][3]
            waypoint.hover_time = item.get("hover_time", self.hover_time)
            waypoint.acceptance_radius = item.get(
                "acceptance_radius", self.acceptance_radius
            )
            self.waypoints.append(waypoint)
        self._publish_markers()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InteractiveWaypointTool()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if bool(node.get_parameter("save_on_shutdown").value):
            node._save_waypoints()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
