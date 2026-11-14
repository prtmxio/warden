"""
bot_cam — ESP32-S3 MJPEG reader + ArUco decode.

Continuously reads the S3 camera stream and publishes detected box ArUco IDs
(10-13) to /box_aruco_detected.  control_node gates on FSM state, so this node
publishes on every new detection without needing to be asked.
"""

import sys
import threading
import time
import urllib.request
from collections import deque, Counter

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import BOX_ARUCO_IDS, S3_STREAM_URL, INSPECT_CONFIRM_FRAMES, INSPECT_CONFIRM_WINDOW

_BOX_ID_SET = set(BOX_ARUCO_IDS)


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


class BotCamNode(Node):
    def __init__(self):
        super().__init__("bot_cam_node")

        self._pub = self.create_publisher(String, "/box_aruco_detected", 10)
        self._img_pub = self.create_publisher(RosImage, "/camera/camera_sensor/image_raw", 5)
        # Liveness heartbeat: control_node gates the inspection sweep on this so a dead S3 stream
        # (common over the laggy link) is never silently counted as a clean "no fault seen" OK.
        self._alive_pub = self.create_publisher(String, "/bot_cam_status", 10)

        # Enhanced detector params for close-range, potentially dark labels
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin      = 3
        params.adaptiveThreshWinSizeMax      = 53
        params.adaptiveThreshWinSizeStep     = 10
        params.polygonalApproxAccuracyRate   = 0.08
        params.minMarkerPerimeterRate        = 0.02
        params.cornerRefinementMethod        = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, params)

        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        self._frame_lock   = threading.Lock()
        self._latest_frame = None
        self._frame_seq    = 0      # bumped per captured frame; detection runs only on NEW frames
        self._seen_seq     = -1     # last frame_seq the detector processed
        self._stop_event   = threading.Event()
        self._last_id      = None   # avoid re-publishing the same ID every tick
        # Per-real-frame history of the nearest box ID (or None). A verdict is only published once
        # the same ID dominates this window — one spurious decode or a neighbour box flickering at
        # the frame edge can't trigger a false read. Window is in REAL frames, not 20 Hz ticks, so
        # it holds up even when the stream crawls at a few fps.
        self._id_history   = deque(maxlen=INSPECT_CONFIRM_WINDOW)

        self._capture_thread = threading.Thread(
            target=self._run_capture, daemon=True
        )
        self._capture_thread.start()

        self.create_timer(1.0 / 20.0, self._detect_tick)

        self.get_logger().info(f"bot_cam_node started — stream: {S3_STREAM_URL}")

    def _run_capture(self):
        # Exponential backoff on reconnect: a flaky/unreachable S3 otherwise hammers the shared
        # hotspot every few seconds and starves the overhead phone stream (which the whole nav
        # loop depends on). Back off 2 s → 15 s cap; reset to 2 s once a frame arrives.
        backoff = 2.0
        while not self._stop_event.is_set():
            try:
                self.get_logger().info(f"Connecting to S3 stream: {S3_STREAM_URL}")
                for frame in _mjpeg_frames(S3_STREAM_URL):
                    if self._stop_event.is_set():
                        return
                    backoff = 2.0
                    with self._frame_lock:
                        self._latest_frame = frame
                        self._frame_seq += 1
            except Exception as e:
                if not self._stop_event.is_set():
                    self.get_logger().warn(
                        f"S3 stream error — reconnecting in {backoff:.0f}s: {e}"
                    )
                    time.sleep(backoff)
                    backoff = min(backoff * 2.0, 15.0)

    def _detect_tick(self):
        with self._frame_lock:
            frame = self._latest_frame
            seq   = self._frame_seq
        if frame is None or seq == self._seen_seq:
            return   # no stream yet, or the capture thread hasn't delivered a new frame since last tick
        self._seen_seq = seq

        # A new frame genuinely arrived from the S3 — heartbeat so control_node knows the stream is
        # live and the inspection sweep is actually looking at something.
        self._alive_pub.publish(String(data="ALIVE"))

        vis_frame = frame.copy()
        gray = self._clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        corners, ids, _ = self._detector.detectMarkers(gray)

        # Pick the NEAREST box marker — the one the bot is docked nose-to-nose with fills the most
        # pixels. Picking the first ID in the list instead grabs whichever neighbour box happens to
        # clip the frame edge, which is how a box got the wrong verdict in testing.
        best_id, best_area = None, 0.0
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis_frame, corners, ids)
            for quad, detected_id in zip(corners, ids.flatten()):
                if detected_id in _BOX_ID_SET:
                    area = cv2.contourArea(quad.reshape(-1, 2))
                    if area > best_area:
                        best_area, best_id = area, int(detected_id)

        self._id_history.append(best_id)
        # Confirm over the window: the dominant non-None ID must appear at least CONFIRM_FRAMES times
        # before we commit to a verdict. Republish only on a change so control_node sees it once.
        counts = Counter(i for i in self._id_history if i is not None)
        confirmed = None
        if counts:
            cand, n = counts.most_common(1)[0]
            if n >= INSPECT_CONFIRM_FRAMES:
                confirmed = cand
        if confirmed is not None:
            if confirmed != self._last_id:
                self._last_id = confirmed
                self._pub.publish(String(data=str(confirmed)))
                self.get_logger().info(
                    f"Box ArUco confirmed: ID {confirmed} "
                    f"({counts[confirmed]}/{len(self._id_history)} frames)"
                )
        else:
            self._last_id = None

        # Publish live camera frame with annotations to ROS topic for dashboard
        img_msg = RosImage()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera_jt"
        img_msg.height, img_msg.width, _ = vis_frame.shape
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = False
        img_msg.step = img_msg.width * 3
        img_msg.data = vis_frame.tobytes()
        self._img_pub.publish(img_msg)

    def destroy_node(self):
        self._stop_event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BotCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
