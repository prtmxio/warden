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

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import BOX_ARUCO_IDS, S3_STREAM_URL

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
        self._stop_event   = threading.Event()
        self._last_id      = None   # avoid re-publishing the same ID every tick

        self._capture_thread = threading.Thread(
            target=self._run_capture, daemon=True
        )
        self._capture_thread.start()

        self.create_timer(1.0 / 20.0, self._detect_tick)

        self.get_logger().info(f"bot_cam_node started — stream: {S3_STREAM_URL}")

    def _run_capture(self):
        while not self._stop_event.is_set():
            try:
                self.get_logger().info(f"Connecting to S3 stream: {S3_STREAM_URL}")
                for frame in _mjpeg_frames(S3_STREAM_URL):
                    if self._stop_event.is_set():
                        return
                    with self._frame_lock:
                        self._latest_frame = frame
            except Exception as e:
                if not self._stop_event.is_set():
                    self.get_logger().warn(f"S3 stream error — reconnecting: {e}")
                    time.sleep(0.5)

    def _detect_tick(self):
        with self._frame_lock:
            frame = self._latest_frame
        if frame is None:
            return

        gray = self._clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        corners, ids, _ = self._detector.detectMarkers(gray)

        if ids is None:
            self._last_id = None
            return

        for detected_id in ids.flatten():
            if detected_id in _BOX_ID_SET:
                if detected_id != self._last_id:
                    self._last_id = detected_id
                    self._pub.publish(String(data=str(detected_id)))
                    self.get_logger().info(f"Box ArUco detected: ID {detected_id}")
                return   # publish only the first box ID per tick

        self._last_id = None

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
