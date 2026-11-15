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
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import (
    ARENA_SIZE_CM, BOT_ARUCO_ID, CORNER_ARUCO_IDS, PHONE_STREAM_URL, WARP_SIZE,
    BOX_HSV_LO, BOX_HSV_HI, BOX_MIN_AREA,
    ARENA_FREE, TOP_ARUCO_IDS, BOT_MARKER_YAW_OFF,
)

# Box centroids closer than this to the bot marker are discarded — the bot (with its
# ArUco standoff) can otherwise show up as a contour blob.
_BOT_EXCLUDE_M = 0.20

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

        # /box_positions is latched (transient_local) so control_node always gets the
        # most recent set of detected box centroids even if it snapshots them late.
        box_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._box_pub = self.create_publisher(PoseArray, "/box_positions", box_qos)

        # Arena-free (pixel-space) topics: raw-image bot pose + box-top marker pixels.
        #   /bot_px : position=(u,v) px, orientation=heading in the image (math-convention)
        #   /box_px : one Pose per visible box-top marker; position=(u,v) px, z=float(ArUco id)
        self._bot_px_pub = self.create_publisher(PoseWithCovarianceStamped, "/bot_px", 10)
        self._box_px_pub = self.create_publisher(PoseArray, "/box_px", 10)

        # /map uses transient_local so Nav2 receives the static map even if it
        # subscribes after the message is first published.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)

        # ArUco detector. Sub-pixel corner refinement is essential here: the bot heading is
        # derived from the marker's corner geometry, and raw (integer-pixel) corners jitter by
        # several degrees frame-to-frame — enough that the control node could never settle an
        # alignment. SUBPIX localises each corner to a fraction of a pixel, cutting that noise.
        aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()
        aruco_params.cornerRefinementMethod     = cv2.aruco.CORNER_REFINE_SUBPIX
        aruco_params.cornerRefinementWinSize     = 5
        aruco_params.cornerRefinementMaxIterations = 30
        aruco_params.cornerRefinementMinAccuracy = 0.01
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self._H           = None   # homography; updated whenever all 4 corners visible
        self._diag_t      = 0.0    # last time a diagnostic heartbeat was logged
        self._frame_lock  = threading.Lock()
        self._latest_frame = None
        self._stop_event  = threading.Event()

        if not ARENA_FREE:
            self._publish_map()

        self._capture_thread = threading.Thread(
            target=self._run_capture, daemon=True
        )
        self._capture_thread.start()

        self.create_timer(1.0 / 15.0, self._process_frame)

        mode = "ARENA-FREE (pixel servo)" if ARENA_FREE else f"warped {ARENA_SIZE_CM}cm arena"
        self.get_logger().info(
            f"vision_node started — stream: {PHONE_STREAM_URL} | mode: {mode}"
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

    def _diag(self, msg: str):
        """Throttled (~1 Hz) status line so you can see what the camera is doing."""
        now = time.time()
        if now - self._diag_t >= 1.0:
            self._diag_t = now
            self.get_logger().info(f"[vision] {msg}")

    def _process_frame(self):
        with self._frame_lock:
            frame = self._latest_frame
        if frame is None:
            self._diag("no frames from phone stream yet — check PHONE_STREAM_URL / hotspot")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        id_list   = [] if ids is None else [int(m) for m in ids.flatten()]

        if ARENA_FREE:
            self._process_frame_px(corners, id_list)
            return

        n_corners = sum(1 for m in id_list if m in CORNER_ARUCO_IDS)

        # Update homography whenever all 4 corners are visible
        new_H, _ = _compute_homography(corners, ids) if ids is not None else (None, None)
        if new_H is not None:
            self._H = new_H

        if self._H is None:
            missing = [c for c in CORNER_ARUCO_IDS if c not in id_list]
            self._diag(
                f"corners {n_corners}/4 — NO WARP yet. Missing corner IDs {missing}. "
                f"All IDs seen now: {sorted(id_list)}"
            )
            return

        # Find bot marker (ID 5) and publish pose
        bot_world = None
        for i, mid in enumerate(id_list):
            if mid != BOT_ARUCO_ID:
                continue
            raw_cx, raw_cy = _marker_center(corners[i])
            wx, wy = _warp_point(self._H, raw_cx, raw_cy)

            # Warped pixel → world metres.  Y is flipped because image Y increases
            # downward but ROS2 map Y increases northward (upward in the arena view).
            x_world = float(wx / PX_PER_CM / 100.0)
            y_world = float((WARP_SIZE - wy) / PX_PER_CM / 100.0)
            yaw     = float(math.radians(_marker_heading(corners[i])))

            bot_world = (x_world, y_world)
            self._publish_pose(x_world, y_world, yaw)
            break

        # Detect box blobs in the warped arena and publish their world positions.
        warped  = cv2.warpPerspective(frame, self._H, (WARP_SIZE, WARP_SIZE))
        n_boxes = self._publish_boxes(warped, bot_world)

        bot_str = (f"bot@({bot_world[0]:.2f},{bot_world[1]:.2f})"
                   if bot_world is not None else "bot NOT seen (ID 5)")
        self._diag(f"WARP-LOCKED (corners 4/4) | {bot_str} | boxes detected: {n_boxes}")

    def _process_frame_px(self, corners, id_list):
        """Arena-free path: publish the bot marker and box-top markers in raw image pixels.
        No homography, no warp — control_node servos one marker toward the other directly.

        Box top ArUco convention: its 'top edge' (forward direction) points TOWARD the
        inspection face of the box. The control node will position the bot on that side,
        approaching with its nose pointing 180° opposite — face-to-face docking."""
        bot_px = None
        for i, mid in enumerate(id_list):
            if mid == BOT_ARUCO_ID:
                u, v = _marker_center(corners[i])
                hdg  = _marker_heading(corners[i]) + BOT_MARKER_YAW_OFF
                bot_px = (u, v, math.radians(hdg))
                self._publish_bot_px(u, v, math.radians(hdg))
                break

        boxes = []
        for i, mid in enumerate(id_list):
            if mid in TOP_ARUCO_IDS:
                u, v   = _marker_center(corners[i])
                hdg    = _marker_heading(corners[i])   # top edge → inspection face direction
                boxes.append((mid, u, v, hdg))
        self._publish_box_px(boxes)

        if bot_px is None:
            self._diag(f"bot NOT seen (ID {BOT_ARUCO_ID}) | box markers: "
                       f"{[b[0] for b in boxes]} | all IDs: {sorted(id_list)}")
        else:
            self._diag(f"bot@px({bot_px[0]:.0f},{bot_px[1]:.0f}) "
                       f"hdg={math.degrees(bot_px[2]):.0f}° | box markers: "
                       f"{[(b[0], int(b[1]), int(b[2]), int(b[3])) for b in boxes]}")

    def _publish_bot_px(self, u: float, v: float, heading: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "image"
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(u)
        msg.pose.pose.position.y = float(v)
        msg.pose.pose.orientation.z = math.sin(heading / 2.0)
        msg.pose.pose.orientation.w = math.cos(heading / 2.0)
        self._bot_px_pub.publish(msg)

    def _publish_box_px(self, boxes):
        """Publish box-top markers.  Each Pose encodes:
          position.x/y = pixel centre (u, v)
          position.z   = float(ArUco id)
          orientation  = box top heading as a planar quaternion
                         (top edge of the printed marker → inspection face direction)
        The control_node uses this heading to compute the face-on approach point so the
        bot arrives with its nose 180° opposite to the box's forward direction."""
        msg = PoseArray()
        msg.header.frame_id = "image"
        msg.header.stamp    = self.get_clock().now().to_msg()
        for bid, u, v, hdg_deg in boxes:
            p = Pose()
            p.position.x = float(u)
            p.position.y = float(v)
            p.position.z = float(bid)        # ArUco id carried in z
            hdg_rad = math.radians(hdg_deg)
            p.orientation.z = math.sin(hdg_rad / 2.0)
            p.orientation.w = math.cos(hdg_rad / 2.0)
            msg.poses.append(p)
        self._box_px_pub.publish(msg)

    def _detect_box_centroids(self, warped_bgr):
        """Return [(x_m, y_m), …] world-metre centroids of box-coloured blobs.
        HSV thresholds + min-area come from arena_config (tuned via contour_test.py)."""
        hsv  = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(BOX_HSV_LO), np.array(BOX_HSV_HI))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for cnt in contours:
            if cv2.contourArea(cnt) < BOX_MIN_AREA:
                continue
            M = cv2.moments(cnt)
            if M["m00"] <= 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            out.append((float(cx / PX_PER_CM / 100.0),
                        float((WARP_SIZE - cy) / PX_PER_CM / 100.0)))
        return out

    def _publish_boxes(self, warped_bgr, bot_world):
        centroids = self._detect_box_centroids(warped_bgr)
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp    = self.get_clock().now().to_msg()
        for x, y in centroids:
            if bot_world is not None and math.hypot(x - bot_world[0], y - bot_world[1]) < _BOT_EXCLUDE_M:
                continue   # this blob is the bot itself, not a box
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.orientation.w = 1.0
            msg.poses.append(p)
        self._box_pub.publish(msg)
        return len(msg.poses)

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
