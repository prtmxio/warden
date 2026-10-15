"""
vision_node — overhead camera: warp, ArUco, /robot_pose, /map.

Background thread reads the phone MJPEG stream.
15 Hz ROS2 timer processes the latest frame, computes homography from corner
ArUcos (IDs 0-3), tracks the bot marker (ID 5), and publishes pose.
A static OccupancyGrid is published once on /map (transient_local so Nav2
picks it up whenever it subscribes).
"""

import math
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import (
    ARENA_SIZE_CM, BOT_ARUCO_ID, CORNER_ARUCO_IDS, PHONE_STREAM_URL, WARP_SIZE,
)

PX_PER_CM = WARP_SIZE / ARENA_SIZE_CM   # px per cm in the warped 800×800 image
ARENA_M   = ARENA_SIZE_CM / 100.0       # arena side in metres

# Maps each corner ArUco ID to its fixed pixel location in the warped output.
#   ID 0 → top-left   (world: x=0,     y=ARENA_M)
#   ID 1 → top-right  (world: x=ARENA_M, y=ARENA_M)
#   ID 2 → bottom-right (world: x=ARENA_M, y=0)
#   ID 3 → bottom-left  (world: x=0,     y=0)
CORNER_DST = {
    0: [0,         0        ],
    1: [WARP_SIZE, 0        ],
    2: [WARP_SIZE, WARP_SIZE],
    3: [0,         WARP_SIZE],
}

# Covariance values — increase if camera is shaky, decrease if stable
_POS_VAR = 0.05 ** 2   # position variance (m²)
_YAW_VAR = 0.05 ** 2   # yaw variance (rad²)


def _mjpeg_frames(url: str, timeout: int = 5):
    """Yield decoded BGR frames from a multipart MJPEG HTTP stream."""
    r = urllib.request.urlopen(url, timeout=timeout)
    buf = b""
    while True:
        buf += r.read(4096)
        a = buf.find(b'\xff\xd8')
        b = buf.find(b'\xff\xd9')
        if a != -1 and b != -1 and b > a:
            jpg = buf[a:b + 2]
            buf = buf[b + 2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                yield frame


def _marker_center(corners_entry):
    pts = corners_entry[0]
    return float(pts[:, 0].mean()), float(pts[:, 1].mean())


def _marker_heading(corners_entry) -> float:
    """Return the marker's forward heading in degrees (math convention, CCW from +x).
    ArUco corners are ordered TL, TR, BR, BL; top-edge → forward direction."""
    pts = corners_entry[0]
    top_mid    = (pts[0] + pts[1]) / 2.0
    bottom_mid = (pts[3] + pts[2]) / 2.0
    dx = top_mid[0] - bottom_mid[0]
    dy = top_mid[1] - bottom_mid[1]
    return float(np.degrees(np.arctan2(-dy, dx)))   # -dy corrects for image Y-down


def _compute_homography(corners, ids):
    """Return (H, src_pts) if all 4 corner ArUcos are visible, else (None, None)."""
    id_to_pos = {}
    for i, mid in enumerate(ids.flatten()):
        if mid in CORNER_ARUCO_IDS:
            id_to_pos[mid] = list(_marker_center(corners[i]))
    if len(id_to_pos) < 4:
        return None, None
    src_pts = np.array([id_to_pos[cid] for cid in CORNER_ARUCO_IDS], dtype=np.float32)
    dst_pts = np.array([CORNER_DST[cid]  for cid in CORNER_ARUCO_IDS], dtype=np.float32)
    H, _ = cv2.findHomography(src_pts, dst_pts)
    return H, src_pts


def _warp_point(H, x: float, y: float):
    pt = np.array([[[x, y]]], dtype=np.float32)
    return cv2.perspectiveTransform(pt, H)[0][0]


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/robot_pose", 10
        )

        # /map uses transient_local so Nav2 receives the static map even if it
        # subscribes after the message is first published.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)

        # ArUco detector — same settings as warp_live_test.py
        aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self._H           = None   # homography; updated whenever all 4 corners visible
        self._frame_lock  = threading.Lock()
        self._latest_frame = None
        self._stop_event  = threading.Event()

        self._publish_map()

        self._capture_thread = threading.Thread(
            target=self._run_capture, daemon=True
        )
        self._capture_thread.start()

        self.create_timer(1.0 / 15.0, self._process_frame)

        self.get_logger().info(
            f"vision_node started — stream: {PHONE_STREAM_URL} | "
            f"arena: {ARENA_SIZE_CM}cm × {ARENA_SIZE_CM}cm ({ARENA_M:.1f}m)"
        )

    def _publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp    = self.get_clock().now().to_msg()
        res = 0.05   # 5 cm grid resolution
        cells = int(ARENA_M / res)
        msg.info.resolution  = res
        msg.info.width       = cells
        msg.info.height      = cells
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * (cells * cells)   # all free
        self._map_pub.publish(msg)
        self.get_logger().info(
            f"/map published — {cells}×{cells} cells at {res*100:.0f}cm resolution"
        )

    def _run_capture(self):
        """Background thread: keep the latest frame fresh."""
        while not self._stop_event.is_set():
            try:
                for frame in _mjpeg_frames(PHONE_STREAM_URL):
                    if self._stop_event.is_set():
                        return
                    with self._frame_lock:
                        self._latest_frame = frame
            except Exception as e:
                if not self._stop_event.is_set():
                    self.get_logger().warn(f"Stream error — reconnecting: {e}")
                    time.sleep(0.5)

    def _process_frame(self):
        with self._frame_lock:
            frame = self._latest_frame
        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        if ids is None:
            return

        # Update homography whenever all 4 corners are visible
        new_H, _ = _compute_homography(corners, ids)
        if new_H is not None:
            self._H = new_H

        if self._H is None:
            return

        # Find bot marker (ID 5) and publish pose
        for i, mid in enumerate(ids.flatten()):
            if mid != BOT_ARUCO_ID:
                continue
            raw_cx, raw_cy = _marker_center(corners[i])
            wx, wy = _warp_point(self._H, raw_cx, raw_cy)

            # Warped pixel → world metres.  Y is flipped because image Y increases
            # downward but ROS2 map Y increases northward (upward in the arena view).
            x_world = float(wx / PX_PER_CM / 100.0)
            y_world = float((WARP_SIZE - wy) / PX_PER_CM / 100.0)
            yaw     = float(math.radians(_marker_heading(corners[i])))

            self._publish_pose(x_world, y_world, yaw)
            break

    def _publish_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 6×6 covariance [x, y, z, roll, pitch, yaw]
        c = msg.pose.covariance
        c[0]  = _POS_VAR   # x
        c[7]  = _POS_VAR   # y
        c[14] = 1e6        # z (2-D mode — ignored by EKF but must be non-zero)
        c[21] = 1e6        # roll
        c[28] = 1e6        # pitch
        c[35] = _YAW_VAR   # yaw

        self._pose_pub.publish(msg)

    def destroy_node(self):
        self._stop_event.set()
        super().destroy_node()


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
