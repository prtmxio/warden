"""
vision_node — overhead camera: warp, ArUco, contour, /robot_pose.

Day 3 skeleton: subscribes/publishes the right topics and spins.
Full implementation goes in on Day 4 (warp_live_test.py logic ported here).
"""

import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import PHONE_STREAM_URL, WARP_SIZE


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/robot_pose", 10
        )
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", 10)

        self.get_logger().info(
            f"vision_node skeleton running. "
            f"Stream URL: {PHONE_STREAM_URL} | warp size: {WARP_SIZE}"
        )
        # Full warp + ArUco + contour logic wired in on Day 4


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
