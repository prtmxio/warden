"""
sim_vision_node — Tier-1.5 sim: real ArUco detection from Gazebo overhead camera.

Subscribes:
  /overhead_camera/image_raw  Image  — Gazebo world camera at (1.0, 1.0, 3.0), pitch=90°

Publishes:
  /robot_pose  PoseWithCovarianceStamped  — bot pose in map frame, derived from ArUco ID 5

Corner IDs 0-3 establish the homography (perspective warp) each frame.
Bot ID 5 is warped through H → converted to Gazebo map coordinates → /robot_pose.
This is the same pipeline as vision_node + warp_live_test.py on real hardware,
but reading from a ROS topic instead of a phone MJPEG URL.
"""

import math
import sys

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped

# arena_config lives in the scada/ dir (shared constants, no hardware deps)
sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import WARP_SIZE, ARENA_SIZE_CM

ARENA_SIZE_M = ARENA_SIZE_CM / 100.0   # 2.0 m
BOT_ID       = 5
CORNER_IDS   = [0, 1, 2, 3]

# Maps each corner ArUco ID to its destination pixel in the 800×800 warped image.
# Must match the physical placement in arena.world:
#   ID 0 → Gazebo (0.075, 1.925) → top-left  of warp → pixel (0,   0  )
#   ID 1 → Gazebo (1.925, 1.925) → top-right of warp → pixel (800, 0  )
#   ID 2 → Gazebo (1.925, 0.075) → bot-right of warp → pixel (800, 800)
#   ID 3 → Gazebo (0.075, 0.075) → bot-left  of warp → pixel (0,   800)
CORNER_DST = {
    0: [0,         0        ],
    1: [WARP_SIZE, 0        ],
    2: [WARP_SIZE, WARP_SIZE],
    3: [0,         WARP_SIZE],
}


class SimVisionNode(Node):
    def __init__(self):
        super().__init__("sim_vision_node")

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/robot_pose", 10
        )

        self.create_subscription(
            Image, "/overhead_camera/image_raw", self._on_image, 5
        )

        self._detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters(),
        )
        self._H = None   # homography — recomputed whenever all 4 corners are visible

        self.get_logger().info(
            f"sim_vision_node ready — listening on /overhead_camera/image_raw "
            f"(WARP_SIZE={WARP_SIZE}, ARENA={ARENA_SIZE_M}m)"
        )

    def _on_image(self, msg: Image):
        # ROS Image → OpenCV BGR without cv_bridge
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        bgr = arr if "bgr" in msg.encoding.lower() else arr[:, :, ::-1]
        bgr = np.ascontiguousarray(bgr)

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        if ids is None:
            return

        id_map = {}
        for i, mid in enumerate(ids.flatten()):
            id_map[int(mid)] = (corners[i], corners[i][0].mean(axis=0))

        # Refresh homography whenever all 4 corner markers are visible
        if all(k in id_map for k in CORNER_IDS):
            src = np.float32([id_map[k][1] for k in CORNER_IDS])
            dst = np.float32([CORNER_DST[k] for k in CORNER_IDS])
            H, _ = cv2.findHomography(src, dst)
            if H is not None:
                self._H = H
                self.get_logger().debug("homography updated")

        if self._H is None or BOT_ID not in id_map:
            return

        bot_corners, bot_center = id_map[BOT_ID]
        cx_raw, cy_raw = float(bot_center[0]), float(bot_center[1])

        # Warp bot pixel into 800×800 arena space
        wp = cv2.perspectiveTransform(
            np.array([[[cx_raw, cy_raw]]], dtype=np.float32), self._H
        )[0][0]

        # Convert warp pixels → Gazebo map metres:
        #   pixel-x 0→800 maps to Gazebo-x 0→ARENA_SIZE_M (west→east)
        #   pixel-y 0→800 maps to Gazebo-y ARENA_SIZE_M→0 (north→south, y-down in image)
        x_m = float(wp[0]) / WARP_SIZE * ARENA_SIZE_M
        y_m = ARENA_SIZE_M - float(wp[1]) / WARP_SIZE * ARENA_SIZE_M

        # Heading from corner geometry of bot marker (same as warp_live_test.marker_heading)
        pts       = bot_corners[0]      # shape (4,2): TL, TR, BR, BL
        top_mid   = (pts[0] + pts[1]) / 2.0
        bot_mid   = (pts[3] + pts[2]) / 2.0
        dx        = float(top_mid[0] - bot_mid[0])
        dy        = float(top_mid[1] - bot_mid[1])
        theta_rad = math.atan2(-dy, dx)   # negate dy: image +y is down, ROS +y is up

        p = PoseWithCovarianceStamped()
        p.header.stamp    = self.get_clock().now().to_msg()
        p.header.frame_id = "map"
        p.pose.pose.position.x    = x_m
        p.pose.pose.position.y    = y_m
        p.pose.pose.orientation.z = math.sin(theta_rad / 2.0)
        p.pose.pose.orientation.w = math.cos(theta_rad / 2.0)
        # Moderate trust (real detection, not ground truth)
        p.pose.covariance[0]  = 0.005
        p.pose.covariance[7]  = 0.005
        p.pose.covariance[35] = 0.01
        self._pose_pub.publish(p)

        self.get_logger().debug(
            f"bot pose: x={x_m:.3f}m  y={y_m:.3f}m  θ={math.degrees(theta_rad):.1f}°"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
