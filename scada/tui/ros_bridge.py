from __future__ import annotations

import math
import queue
import sys
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from scada.tui.app import WardenApp

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Range
    from sensor_msgs.msg import Image as RosImage
    from geometry_msgs.msg import PoseWithCovarianceStamped
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Node = object  # type: ignore[assignment,misc]

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
try:
    from arena_config import PHONE_STREAM_URL, S3_STREAM_URL
except ImportError:
    PHONE_STREAM_URL = ""
    S3_STREAM_URL = ""

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class _NullNode:
    """Stand-in when ROS2 is not available (smoke-test / unit-test mode)."""

    def publish_decision(self, payload: str) -> None:
        pass

    def publish_stop(self) -> None:
        pass

    def destroy_node(self) -> None:
        pass


if HAS_ROS:
    class TuiRosNode(Node):  # type: ignore[misc]
        def __init__(self, app: "WardenApp", *, sim: bool = False) -> None:
            super().__init__("tui_node")
            self._app = app
            self._sim = sim
            self._current_state = "SETUP"

            self._overhead_q: queue.Queue = queue.Queue(maxsize=1)
            self._botcam_q: queue.Queue = queue.Queue(maxsize=1)

            self._decision_pub = self.create_publisher(String, "/operator_decision", 10)
            self._cmd_pub      = self.create_publisher(String, "/bot_command", 10)

            self.create_subscription(String, "/fsm_state",         self._on_fsm,     10)
            self.create_subscription(String, "/inspection_result", self._on_inspect,  10)
            self.create_subscription(
                PoseWithCovarianceStamped, "/robot_pose", self._on_pose, 10
            )
            self.create_subscription(Range, "/tof_range", self._on_tof, 10)
            self.create_subscription(
                RosImage, "/camera/camera_sensor/image_raw", self._on_botcam_img, 5
            )

            if sim:
                self.create_subscription(
                    RosImage, "/overhead_camera/image_raw", self._on_overhead_img, 5
                )

            threading.Thread(target=self._overhead_worker, daemon=True).start()
            threading.Thread(target=self._botcam_worker,   daemon=True).start()
            if not sim:
                threading.Thread(target=self._phone_worker, daemon=True).start()

            self.get_logger().info("tui_node ready")

        def _on_fsm(self, msg: String) -> None:
            state = msg.data.strip()
            self._current_state = state
            self._app.call_from_thread(self._app.on_fsm_state, state)

        def _on_inspect(self, msg: String) -> None:
            self._app.call_from_thread(self._app.on_inspection_result, msg.data.strip())

        def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny, cosy)
            self._app.call_from_thread(self._app.on_pose, p.x, p.y, theta)

        def _on_tof(self, msg: Range) -> None:
            self._app.call_from_thread(self._app.on_tof, msg.range * 1000.0)

        def _on_overhead_img(self, msg: RosImage) -> None:
            _q_put(self._overhead_q, msg)

        def _on_botcam_img(self, msg: RosImage) -> None:
            _q_put(self._botcam_q, msg)

        def _overhead_worker(self) -> None:
            if not HAS_CV2:
                return
            import numpy as np
            while True:
                msg = self._overhead_q.get()
                try:
                    arr = (
                        np.frombuffer(msg.data, dtype=np.uint8)
                        .reshape(msg.height, msg.width, 3)
                    )
                    bgr = arr.copy() if "bgr" in msg.encoding.lower() else arr[:, :, ::-1].copy()
                    ascii_str = _bgr_to_ascii(bgr)
                    self._app.call_from_thread(
                        self._app.on_overhead_frame, ascii_str
                    )
                except Exception:
                    pass

        def _botcam_worker(self) -> None:
            if not HAS_CV2:
                return
            import numpy as np
            while True:
                msg = self._botcam_q.get()
                if self._current_state != "INSPECTING":
                    continue
                try:
                    arr = (
                        np.frombuffer(msg.data, dtype=np.uint8)
                        .reshape(msg.height, msg.width, 3)
                    )
                    bgr = arr.copy() if "bgr" in msg.encoding.lower() else arr[:, :, ::-1].copy()
                    ascii_str = _bgr_to_ascii(bgr)
                    self._app.call_from_thread(self._app.on_botcam_frame, ascii_str)
                except Exception:
                    pass

        def _phone_worker(self) -> None:
            if not HAS_CV2 or not PHONE_STREAM_URL:
                return
            import cv2 as cv
            cap = None
            while True:
                try:
                    if cap is None or not cap.isOpened():
                        cap = cv.VideoCapture(PHONE_STREAM_URL)
                    ret, frame = cap.read()
                    if not ret:
                        cap.release()
                        cap = None
                        import time; time.sleep(0.5)
                        continue
                    ascii_str = _bgr_to_ascii(frame)
                    self._app.call_from_thread(self._app.on_overhead_frame, ascii_str)
                except Exception:
                    import time; time.sleep(0.5)

        def publish_decision(self, payload: str) -> None:
            msg = String()
            msg.data = payload
            self._decision_pub.publish(msg)

        def publish_stop(self) -> None:
            msg = String()
            msg.data = "STOP"
            self._cmd_pub.publish(msg)

        def destroy_node(self) -> None:  # type: ignore[override]
            super().destroy_node()

else:
    TuiRosNode = _NullNode  # type: ignore[assignment,misc]


def _q_put(q: queue.Queue, item: object) -> None:
    if q.full():
        try:
            q.get_nowait()
        except Exception:
            pass
    try:
        q.put_nowait(item)
    except Exception:
        pass


def _bgr_to_ascii(bgr_frame, cols: int = 60, rows: int = 20) -> str:
    if not HAS_CV2:
        return ""
    sys.path.insert(0, "/home/xio/Desktop/warden/scada")
    from ui.art.dither import bayer_dither
    return bayer_dither(bgr_frame, cols=cols, rows=rows)
