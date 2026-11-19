"""
gui_node — PyQt5 SCADA dashboard (4-panel layout).
Run via: uv run scada/main.py  (or standalone: uv run scada/gui_node.py)

Panels:
  Top-left:  Phone cam feed (overhead MJPEG)
  Top-right: Bot cam feed (S3 MJPEG — active only during INSPECTING)
  Sidebar:   State badge, TOF, bot pose, box status grid, START/STOP

Thread model:
  Main thread  → Qt event loop, all widget updates
  ros_thread   → SingleThreadedExecutor.spin() for GuiNode
  phone_cam    → QThread reading MJPEG, emits QImage via signal
  bot_cam      → QThread reading S3 MJPEG, emits QImage via signal
"""

import math
import os
import sys
import threading

# ── import order: PyQt5 C extensions MUST load before cv2 ────────────────────
# cv2 bundles its own Qt .so files (libQt5Core.so.5, libQt5Gui.so.5, …).
# Whichever package is dlopen'd first wins the linker cache for that soname —
# subsequent loaders reuse what's already in memory.  If cv2 loads first, its
# Qt5Core.so becomes the active Qt runtime, and the platform-plugin search
# path it has compiled in (/…/cv2/qt/plugins) takes effect, making xcb fail.
# Loading PyQt5's C extensions first ensures PyQt5's Qt .so files win, and
# cv2 simply reuses them — no ABI mismatch, no plugin-path conflict.

# Step 1 — force the env var NOW (before any .so is dlopen'd).
try:
    import PyQt5 as _pyqt5_pkg
    _QT_PLUGINS = os.path.join(os.path.dirname(_pyqt5_pkg.__file__), "Qt5", "plugins")
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _QT_PLUGINS   # force, not setdefault
    del _pyqt5_pkg
except ImportError:
    _QT_PLUGINS = None

# Step 2 — load PyQt5 C extensions (pulls Qt5Core/Qt5Widgets.so into memory).
try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QLabel, QDialog,
        QPushButton, QVBoxLayout, QHBoxLayout, QSizePolicy, QFrame,
        QSplitter, QListWidget, QListWidgetItem, QAbstractItemView,
        QLCDNumber,
    )
    from PyQt5.QtCore import (
        Qt, QThread, pyqtSignal, QObject, QTimer, QCoreApplication,
        QRect,
    )
    from PyQt5.QtGui import (
        QImage, QPixmap, QColor, QPainter, QPen, QBrush, QFont,
        QPalette,
    )
    # Belt-and-suspenders: also register the path via Qt's own API so it
    # takes precedence over any path baked into a shared library.
    if _QT_PLUGINS:
        QCoreApplication.addLibraryPath(_QT_PLUGINS)
    HAS_QT = True
except ImportError:
    HAS_QT = False
    class _Stub:
        Expanding = HLine = AlignCenter = AlignLeft = KeepAspectRatio = None
        SmoothTransformation = DotLine = Bold = Format_RGB888 = None
        def __init__(self, *a, **kw): pass
        def __call__(self, *a, **kw): return self
    _s = _Stub()
    (QObject, QThread, QWidget, QMainWindow, QDialog, QTimer,
     QApplication, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
     QSizePolicy, QFrame, QSplitter, QImage, QPixmap, QColor, QPainter, QPen,
     QBrush, QFont, QListWidget, QListWidgetItem, QAbstractItemView,
     QLCDNumber, QRect, QPalette) = (_Stub,) * 27
    Qt = _s
    def pyqtSignal(*a, **kw): return None  # noqa: E306

# Step 3 — ROS2 (order doesn't matter for rclpy).
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
from sensor_msgs.msg import Image as RosImage
from geometry_msgs.msg import PoseWithCovarianceStamped

# Step 4 — cv2 last; by now PyQt5's Qt .so files are in the linker cache,
# so cv2 reuses them and its bundled copies are never loaded.
try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import (
    PHONE_STREAM_URL, S3_STREAM_URL,
    BOX_POSITIONS, BOX_ARUCO_IDS, ARENA_SIZE_CM,
)

# ── visual constants ──────────────────────────────────────────────────────────

BG_0   = "#0a0a0c"
BG_1   = "#101014"
BG_2   = "#16161c"
LINE   = "#2a2a33"
INK    = "#d7d7cf"
DIM    = "#6b6b73"
AMBER  = "#ff8c1a"
OK     = "#3e8e5a"
FAULT  = "#c0392b"
FONT   = '"JetBrains Mono","DejaVu Sans Mono","Consolas",monospace'

STATE_COLORS = {
    "SETUP":            DIM,
    "IDLE":             INK,
    "NAVIGATING":       INK,
    "ALIGNING":         INK,
    "APPROACH_SCAN":    INK,
    "APPROACH_DOCK":    INK,
    "DOCKING":          INK,
    "INSPECTING":       INK,
    "AWAITING_HUMAN":   AMBER,
    "REVIEWING_QUEUE":  INK,
    "REPAIRING":        AMBER,
    "RETURNING_HOME":   INK,
    "MISSION_COMPLETE": OK,
}

BOX_STATUS_STYLES = {
    "UNKNOWN":  f"background:{BG_2}; color:{DIM}; border:1px solid {LINE};",
    "OK":       f"background:{BG_2}; color:{OK}; border:1px solid {OK};",
    "FAULT":    f"background:{BG_2}; color:{FAULT}; border:1px solid {FAULT};",
    "QUEUED":   f"background:{BG_2}; color:{DIM}; border:1px solid {LINE};",
    "REPAIRED": f"background:{BG_2}; color:{INK}; border:1px solid {LINE};",
    "SKIPPED":  f"background:{BG_2}; color:{DIM}; border:1px dashed {AMBER};",
}


# ── thread-safe signal bridge ─────────────────────────────────────────────────

class RosBridge(QObject):
    if HAS_QT:
        sig_fsm_state   = pyqtSignal(str)
        sig_inspection  = pyqtSignal(str)
        sig_bot_pose    = pyqtSignal(float, float, float)
        sig_tof         = pyqtSignal(float)
        sig_phone_frame = pyqtSignal(QImage)   # raw overhead cam → top-left panel
        sig_warp_frame  = pyqtSignal(QImage)   # warped overhead cam → arena blueprint background
        sig_bot_frame   = pyqtSignal(QImage)   # used in --sim mode for bot forward cam
        sig_tof_frame   = pyqtSignal(QImage)   # depth visualizer


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class GuiNode(Node):
    def __init__(self, bridge: RosBridge):
        super().__init__("gui_node")
        self._bridge = bridge

        import queue
        # 1-slot "latest frame" queues — ROS callbacks are non-blocking drops;
        # worker threads do the heavy cv2 work without touching the spin thread.
        self._overhead_q = queue.Queue(maxsize=1)
        self._botcam_q   = queue.Queue(maxsize=1)

        threading.Thread(target=self._overhead_worker, daemon=True).start()
        threading.Thread(target=self._botcam_worker,   daemon=True).start()

        self._decision_pub = self.create_publisher(String, "/operator_decision", 10)
        self._cmd_pub      = self.create_publisher(String, "/bot_command",       10)

        self.create_subscription(String, "/fsm_state",
                                 self._on_fsm, 10)
        self.create_subscription(String, "/inspection_result",
                                 self._on_inspection, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/robot_pose",
                                 self._on_pose, 10)
        self.create_subscription(Range, "/tof_range",
                                 self._on_tof, 10)

        # Always subscribe to bot camera frames (published by bot_cam_node or simulation)
        self.create_subscription(RosImage, "/camera/camera_sensor/image_raw",
                                 self._on_bot_cam_img, 5)

        if "--sim" in sys.argv:
            self.create_subscription(RosImage, "/overhead_camera/image_raw",
                                     self._on_overhead_img, 5)
            self.create_subscription(RosImage, "/tof_sensor/depth_sensor/depth/image_raw",
                                     self._on_tof_depth_img, 5)

        self.get_logger().info("gui_node ready")

    def _on_fsm(self, msg: String):
        self._bridge.sig_fsm_state.emit(msg.data.strip())

    def _on_inspection(self, msg: String):
        self._bridge.sig_inspection.emit(msg.data.strip())

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._bridge.sig_bot_pose.emit(p.x, p.y, math.atan2(siny, cosy))

    def _on_tof(self, msg: Range):
        self._bridge.sig_tof.emit(msg.range * 1000.0)

    # ROS callbacks — just drop the latest frame into the queue; never block spin thread
    def _on_overhead_img(self, msg: RosImage):
        self._q_put(self._overhead_q, msg)

    def _on_bot_cam_img(self, msg: RosImage):
        self._q_put(self._botcam_q, msg)

    @staticmethod
    def _q_put(q, item):
        """Non-blocking latest-frame drop: evict old if full, then insert."""
        if q.full():
            try: q.get_nowait()
            except Exception: pass
        try: q.put_nowait(item)
        except Exception: pass

    # Worker threads — do all heavy cv2 work here, ROS spin thread stays free
    def _overhead_worker(self):
        import numpy as np, cv2
        WARP_SIZE = 800
        while True:
            msg = self._overhead_q.get()   # blocks until a frame arrives
            try:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                bgr = arr.copy() if "bgr" in msg.encoding.lower() else arr[:, :, ::-1].copy()

                if '--sim' in sys.argv:
                    if not hasattr(self, '_sim_H'):
                        fov = 0.8
                        px_per_m = ((msg.width / 2) / math.tan(fov / 2)) / 3.0
                        cx, cy, cam_x, cam_y = msg.width / 2, msg.height / 2, 1.0, 1.0
                        mxy = [(0.075, 1.925), (1.925, 1.925), (1.925, 0.075), (0.075, 0.075)]
                        src = np.float32([
                            [cx - (y - cam_y) * px_per_m, cy - (x - cam_x) * px_per_m]
                            for x, y in mxy
                        ])
                        dst = np.float32([[0, 0], [WARP_SIZE, 0], [WARP_SIZE, WARP_SIZE], [0, WARP_SIZE]])
                        self._sim_H, _ = cv2.findHomography(src, dst)
                    out = cv2.warpPerspective(bgr, self._sim_H, (WARP_SIZE, WARP_SIZE))
                else:
                    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
                    corners, ids, _ = detector.detectMarkers(gray)
                    out = bgr
                    if ids is not None:
                        id_map = {}
                        for c, mid in zip(corners, ids.flatten()):
                            if int(mid) in (0, 1, 2, 3):
                                id_map[int(mid)] = c[0].mean(axis=0)
                        if all(k in id_map for k in (0, 1, 2, 3)):
                            src = np.float32([id_map[k] for k in (0, 1, 2, 3)])
                            dst = np.float32([[0, 0], [WARP_SIZE, 0],
                                               [WARP_SIZE, WARP_SIZE], [0, WARP_SIZE]])
                            H, _ = cv2.findHomography(src, dst)
                            if H is not None:
                                out = cv2.warpPerspective(bgr, H, (WARP_SIZE, WARP_SIZE))

                raw_rgb = np.ascontiguousarray(bgr[:, :, ::-1])
                rh, rw = raw_rgb.shape[:2]
                raw_qi = QImage(raw_rgb.tobytes(), rw, rh, 3 * rw, QImage.Format_RGB888)
                self._bridge.sig_phone_frame.emit(raw_qi.copy())

                warp_rgb = np.ascontiguousarray(out[:, :, ::-1])
                wh, ww = warp_rgb.shape[:2]
                warp_qi = QImage(warp_rgb.tobytes(), ww, wh, 3 * ww, QImage.Format_RGB888)
                self._bridge.sig_warp_frame.emit(warp_qi.copy())
            except Exception as e:
                self.get_logger().error(f"overhead_worker: {e}")

    def _botcam_worker(self):
        import numpy as np
        while True:
            msg = self._botcam_q.get()
            try:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                rgb = np.ascontiguousarray(arr if "rgb" in msg.encoding.lower() else arr[:, :, ::-1])
                qi = QImage(rgb.tobytes(), msg.width, msg.height, 3 * msg.width, QImage.Format_RGB888)
                self._bridge.sig_bot_frame.emit(qi.copy())
            except Exception as e:
                self.get_logger().error(f"botcam_worker: {e}")

    def _on_tof_depth_img(self, msg: RosImage):
        if not hasattr(self, '_last_tof_time'):
            self._last_tof_time = 0.0
        import time
        now = time.time()
        if now - self._last_tof_time < 0.1:
            return
        self._last_tof_time = now

        import numpy as np
        try:
            if msg.encoding == '32FC1':
                arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            elif msg.encoding == '16UC1':
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
            else:
                return

            # Camera intrinsics — 640×480, H_FOV = 1.047 rad (60°), square pixels
            H_FOV = 1.047
            fx = (msg.width / 2.0) / math.tan(H_FOV / 2.0)
            cx = msg.width  / 2.0
            cy = msg.height / 2.0

            # Subsample for performance: 80×60 → ~4.8 k pts max
            step = 8
            sub = arr[::step, ::step]
            h_s, w_s = sub.shape
            cols = np.tile(np.arange(0, msg.width,  step, dtype=np.float32), (h_s, 1))
            valid = (sub > 0.05) & (sub < 2.5) & np.isfinite(sub)
            d_v = sub[valid]
            u_v = cols[valid]

            # Project to bird's-eye: x_cam=lateral, z_cam=forward depth
            x_cam = (u_v - cx) * d_v / fx
            z_cam = d_v

            # Canvas: 480×270, bot at bottom-centre, 2.5 m fills height
            w_c, h_c, margin = 480, 270, 15
            scale = float(h_c - margin) / 2.5
            origin_x = w_c // 2
            origin_y = h_c - margin

            px = (origin_x - x_cam * scale).astype(int)
            py = (origin_y - z_cam * scale).astype(int)
            in_b = (px >= 0) & (px < w_c) & (py >= 0) & (py < h_c)
            px, py = px[in_b], py[in_b]
            z_v = z_cam[in_b]

            # Build canvas with numpy (grey background like a costmap)
            canvas = np.full((h_c, w_c, 3), [200, 200, 200], dtype=np.uint8)

            # Square grid every 0.5 m (darker grey lines)
            grid_col = [160, 160, 160]
            for gm in (0.5, 1.0, 1.5, 2.0, 2.5):
                gp = int(gm * scale)
                gy = origin_y - gp
                if 0 <= gy < h_c:
                    canvas[gy, :] = grid_col
                for gx in (origin_x + gp, origin_x - gp):
                    if 0 <= gx < w_c:
                        canvas[:, gx] = grid_col

            # Depth gradient: close → dark crimson, far → bright orange-red
            t = np.clip((z_v - 0.05) / (2.5 - 0.05), 0.0, 1.0)
            r_ch = (60  + t * 195).astype(np.uint8)
            g_ch = (5   + t *  75).astype(np.uint8)
            b_ch = (5   + t *  35).astype(np.uint8)
            colors = np.stack([r_ch, g_ch, b_ch], axis=1)

            for dy, dx in [(0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)]:
                canvas[
                    np.clip(py + dy, 0, h_c - 1),
                    np.clip(px + dx, 0, w_c - 1),
                ] = colors

            # Bot marker — cyan rectangle at sensor origin
            canvas[origin_y - 8:origin_y, origin_x - 5:origin_x + 5] = [0, 160, 220]

            qi = QImage(canvas.tobytes(), w_c, h_c, 3 * w_c, QImage.Format_RGB888)
            self._bridge.sig_tof_frame.emit(qi.copy())

        except Exception as e:
            self.get_logger().error(f"TOF depth visualization failed: {e}")

    def publish_decision(self, decision: str):
        self._decision_pub.publish(String(data=decision))
        self.get_logger().info(f"→ /operator_decision: {decision}")

    def publish_stop(self):
        self._cmd_pub.publish(String(data="STOP"))
        self.get_logger().info("→ /bot_command: STOP")


# ── MJPEG camera thread ───────────────────────────────────────────────────────

class CameraThread(QThread):
    frame_ready = pyqtSignal(QImage)

    def __init__(self, url: str, fps: int = 20):
        super().__init__()
        self._url          = url
        self._interval_ms  = max(1, int(1000 / fps))
        self._running      = False

    def run(self):
        self._running = True
        while self._running:
            cap = None
            if HAS_CV2:
                cap = cv2.VideoCapture(self._url)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            if not HAS_CV2 or cap is None or not cap.isOpened():
                self.frame_ready.emit(self._placeholder())
                self.msleep(2000)
                if cap:
                    cap.release()
                continue

            while self._running and cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    self.frame_ready.emit(self._placeholder())
                    break
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                img = QImage(rgb.tobytes(), w, h, ch * w, QImage.Format_RGB888)
                self.frame_ready.emit(img.copy())
                self.msleep(self._interval_ms)

            if cap:
                cap.release()
            if self._running:
                self.msleep(1000)   # reconnect delay

    @staticmethod
    def _placeholder() -> QImage:
        img = QImage(480, 270, QImage.Format_RGB888)
        img.fill(QColor(10, 10, 12))
        if HAS_QT:
            painter = QPainter(img)
            painter.setPen(QColor(107, 107, 115))
            font = QFont("JetBrains Mono", 11)
            painter.setFont(font)
            painter.drawText(img.rect(), Qt.AlignCenter, "CAMERA OFFLINE\nWAITING FOR SIGNAL")
            painter.end()
        return img

    def stop(self):
        self._running = False
        self.wait(3000)


# ── pixel-art robot sprite ────────────────────────────────────────────────────

class RobotSprite(QWidget):
    """96x96 pixel-art robot drawn with filled QRects; heading tick rotates with theta."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._theta = 0.0
        self.setFixedSize(96, 96)
        self.setAttribute(Qt.WA_TranslucentBackground)

    def set_theta(self, theta: float):
        self._theta = theta
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, False)

        cx, cy = 48, 48
        amber  = QColor(0xff, 0x8c, 0x1a)
        dim    = QColor(0x6b, 0x6b, 0x73)
        dark   = QColor(0x2a, 0x2a, 0x33)
        ink    = QColor(0xd7, 0xd7, 0xcf)

        def px(x, y, w, h, color):
            painter.fillRect(QRect(x, y, w, h), color)

        # chassis body
        px(28, 36, 40, 28, amber)
        # chassis shading (right side, bottom)
        px(60, 36, 8,  28, QColor(0xcc, 0x6a, 0x10))
        px(28, 56, 40,  8, QColor(0xcc, 0x6a, 0x10))
        # head
        px(34, 24, 28, 14, amber)
        px(56, 24,  6, 14, QColor(0xcc, 0x6a, 0x10))
        # eyes
        px(38, 28,  6,  5, dark)
        px(50, 28,  6,  5, dark)
        px(39, 29,  4,  3, ink)
        px(51, 29,  4,  3, ink)
        # left wheel
        px(20, 40, 10, 20, dim)
        px(20, 44,  4, 12, dark)
        # right wheel
        px(66, 40, 10, 20, dim)
        px(72, 44,  4, 12, dark)
        # antenna
        px(47, 16,  2, 10, dim)
        px(44, 14,  8,  4, dim)
        # chest sensor strip
        px(34, 44, 28,  4, dark)
        px(36, 44,  4,  4, QColor(0x3e, 0x8e, 0x5a))
        px(44, 44,  4,  4, amber)

        # heading tick — a short line from centre outward in the direction of theta
        painter.setPen(QPen(amber, 2))
        tick_len = 28
        dx = int(tick_len * math.cos(self._theta))
        dy = int(-tick_len * math.sin(self._theta))
        painter.drawLine(cx, cy, cx + dx, cy + dy)

        painter.end()


# ── dot-matrix stat tile ──────────────────────────────────────────────────────

class SegTile(QWidget):
    """Bordered stat tile: small DIM caption above a QLCDNumber with AMBER segments."""

    def __init__(self, caption: str, digits: int = 6, parent=None):
        super().__init__(parent)
        self.setStyleSheet(
            f"background:{BG_1}; border:1px solid {LINE};"
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(2)

        cap_lbl = QLabel(caption.upper())
        cap_lbl.setAlignment(Qt.AlignCenter)
        cap_lbl.setStyleSheet(
            f"background:transparent; border:none; color:{DIM};"
            f"font-family:{FONT}; font-size:9px; letter-spacing:1px;"
        )
        layout.addWidget(cap_lbl)

        self._lcd = QLCDNumber(digits)
        self._lcd.setSegmentStyle(QLCDNumber.Flat)
        self._lcd.setFrameShape(QFrame.NoFrame)
        self._lcd.setFixedHeight(36)

        palette = self._lcd.palette()
        palette.setColor(QPalette.WindowText, QColor(AMBER))
        palette.setColor(QPalette.Background, QColor(BG_2))
        palette.setColor(QPalette.Dark,       QColor(BG_2))
        palette.setColor(QPalette.Light,      QColor(BG_2))
        self._lcd.setPalette(palette)
        self._lcd.setAutoFillBackground(True)
        self._lcd.setStyleSheet(f"background:{BG_2}; border:none;")
        layout.addWidget(self._lcd)

    def display(self, value):
        self._lcd.display(value)


# ── repair modal ──────────────────────────────────────────────────────────────

_BTN_BASE = (
    f"QPushButton {{"
    f"  background:{BG_2}; color:{INK};"
    f"  font-family:{FONT}; font-size:13px; font-weight:bold;"
    f"  padding:10px; border:1px solid {LINE}; border-radius:0px;"
    f"}}"
    f"QPushButton:hover {{ border-color:{AMBER}; color:{AMBER}; }}"
)

_BTN_PRIMARY = (
    f"QPushButton {{"
    f"  background:{AMBER}; color:{BG_0};"
    f"  font-family:{FONT}; font-size:13px; font-weight:bold;"
    f"  padding:10px; border:1px solid {AMBER}; border-radius:0px;"
    f"}}"
    f"QPushButton:hover {{ background:#ffaa50; border-color:#ffaa50; }}"
)

_BTN_STOP = (
    f"QPushButton {{"
    f"  background:{BG_2}; color:{FAULT};"
    f"  font-family:{FONT}; font-size:13px; font-weight:bold;"
    f"  padding:10px; border:1px solid {FAULT}; border-radius:0px;"
    f"}}"
    f"QPushButton:hover {{ background:{FAULT}; color:{INK}; }}"
)

_BTN_DISABLED = (
    f"QPushButton:disabled {{ background:{BG_1}; color:{DIM}; border:1px solid {LINE}; }}"
)


class RepairDialog(QDialog):
    def __init__(self, box_id: int, node: GuiNode, parent=None):
        super().__init__(parent)
        self._node   = node

        self.setWindowTitle("FAULT DETECTED")
        self.setModal(True)
        self.setMinimumWidth(360)
        self.setStyleSheet(
            f"QDialog {{ background:{BG_1}; border:1px solid {FAULT}; }}"
            f"QLabel {{ color:{INK}; font-family:{FONT}; border:none; }}"
        )

        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(20, 20, 20, 20)

        title = QLabel(f"FAULT DETECTED — BOX {box_id}")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            f"font-size:14px; font-weight:bold; color:{FAULT};"
            f"padding:8px; background:{BG_2}; border:1px solid {FAULT};"
            f"font-family:{FONT};"
        )
        layout.addWidget(title)

        for label, decision, style in [
            ("REPAIR NOW",      "REPAIR_NOW",  _BTN_PRIMARY),
            ("QUEUE FOR LATER", "QUEUE",       _BTN_BASE),
            ("SKIP",            "SKIP",        _BTN_BASE),
        ]:
            btn = QPushButton(label)
            btn.setStyleSheet(style)
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda checked, d=decision: self._decide(d))
            layout.addWidget(btn)

    def _decide(self, decision: str):
        self._node.publish_decision(decision)
        self.accept()


# ── repair queue dialog ───────────────────────────────────────────────────────

class RepairQueueDialog(QDialog):
    def __init__(self, queued_boxes: list, node, parent=None):
        super().__init__(parent)
        self._node = node
        self._selected_order = []

        self.setWindowTitle("MANAGE REPAIR QUEUE")
        self.setModal(True)
        self.setMinimumWidth(450)
        self.setStyleSheet(
            f"QDialog {{ background:{BG_1}; border:1px solid {LINE}; }}"
            f"QLabel {{ color:{INK}; font-family:{FONT}; font-size:13px; border:none; }}"
        )

        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(20, 20, 20, 20)

        title = QLabel("MANAGE REPAIR QUEUE")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            f"font-size:14px; font-weight:bold; color:{AMBER};"
            f"padding:8px; background:{BG_2}; border:1px solid {AMBER};"
            f"font-family:{FONT};"
        )
        layout.addWidget(title)

        if not queued_boxes:
            lbl = QLabel("No boxes currently in queue.")
            lbl.setAlignment(Qt.AlignCenter)
            layout.addWidget(lbl)
            btn = QPushButton("CLOSE")
            btn.setStyleSheet(_BTN_BASE)
            btn.clicked.connect(self.reject)
            layout.addWidget(btn)
            return

        lbl = QLabel("Select boxes to add to execution order:")
        lbl.setStyleSheet(f"color:{DIM}; font-size:11px; font-family:{FONT};")
        layout.addWidget(lbl)

        self._box_layout = QHBoxLayout()
        for bid in queued_boxes:
            btn = QPushButton(f"BOX {bid}")
            btn.setStyleSheet(_BTN_BASE)
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda checked, b=bid: self._toggle_box(b))
            self._box_layout.addWidget(btn)
        layout.addLayout(self._box_layout)

        self._order_lbl = QLabel("ORDER: [NONE]")
        self._order_lbl.setStyleSheet(
            f"color:{AMBER}; font-weight:bold; font-family:{FONT}; font-size:11px;"
        )
        layout.addWidget(self._order_lbl)

        btn_layout = QHBoxLayout()
        self._skip_btn = QPushButton("SKIP ALL & FINISH")
        self._skip_btn.setStyleSheet(_BTN_STOP)
        self._skip_btn.setCursor(Qt.PointingHandCursor)
        self._skip_btn.clicked.connect(self._finish)

        self._exec_btn = QPushButton("EXECUTE SELECTED")
        self._exec_btn.setStyleSheet(_BTN_PRIMARY)
        self._exec_btn.setCursor(Qt.PointingHandCursor)
        self._exec_btn.clicked.connect(self._submit)

        btn_layout.addWidget(self._skip_btn)
        btn_layout.addWidget(self._exec_btn)
        layout.addLayout(btn_layout)

    def _toggle_box(self, box_id):
        if box_id in self._selected_order:
            self._selected_order.remove(box_id)
        else:
            self._selected_order.append(box_id)

        for i in range(self._box_layout.count()):
            btn = self._box_layout.itemAt(i).widget()
            bid_str = btn.text().split("\n")[0].replace("BOX ", "")
            try:
                bid = int(bid_str)
            except Exception:
                continue

            if bid in self._selected_order:
                idx = self._selected_order.index(bid) + 1
                btn.setText(f"BOX {bid}\n#{idx}")
                btn.setStyleSheet(
                    f"QPushButton {{ background:{AMBER}; color:{BG_0};"
                    f"  font-family:{FONT}; font-size:12px; font-weight:bold;"
                    f"  padding:8px; border:1px solid {AMBER}; border-radius:0px; }}"
                    f"QPushButton:hover {{ background:#ffaa50; }}"
                )
            else:
                btn.setText(f"BOX {bid}")
                btn.setStyleSheet(_BTN_BASE)

        if self._selected_order:
            self._order_lbl.setText(
                "ORDER: " + " → ".join(f"BOX {b}" for b in self._selected_order)
            )
        else:
            self._order_lbl.setText("ORDER: [NONE]")

    def _submit(self):
        if not self._selected_order:
            return
        order_str = ",".join(str(x) for x in self._selected_order)
        self._node.publish_decision(f"REPAIR_ORDER:{order_str}")
        self.accept()

    def _finish(self):
        self._node.publish_decision("REPAIR_ORDER:NONE")
        self.accept()


# ── main window ───────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self, bridge: RosBridge, node: GuiNode):
        super().__init__()
        self._bridge        = bridge
        self._node          = node
        self._fsm_state     = "SETUP"
        self._repair_dialog = None
        self._blink_on      = True
        self._box_status    = {bid: "UNKNOWN" for bid in BOX_ARUCO_IDS}

        # bottom-bar live telemetry cache
        self._bar_tof_mm  = 0.0
        self._bar_theta   = 0.0

        self.setWindowTitle("Warden SCADA")
        self.resize(1300, 760)
        self.setStyleSheet(
            f"QMainWindow, QWidget {{ background:{BG_0}; color:{INK}; font-family:{FONT}; }}"
            f"QFrame[frameShape='4'] {{ background:{LINE}; border:none; max-height:1px; }}"
            f"QSplitter::handle {{ background:{LINE}; }}"
        )

        self._build_ui()
        self._connect_bridge()

        self._phone_cam = None
        if "--sim" in sys.argv:
            bridge.sig_phone_frame.connect(self._on_phone_frame)
        else:
            self._phone_cam = CameraThread(PHONE_STREAM_URL, fps=20)
            self._phone_cam.frame_ready.connect(self._on_phone_frame)
            self._phone_cam.start()

        # Bot cam feed is streamed directly via ROS /camera/camera_sensor/image_raw
        # published by bot_cam_node (hardware) or gazebo (sim), avoiding dual-stream collisions.
        bridge.sig_bot_frame.connect(self._on_bot_frame)

        self._blink_timer = QTimer()
        self._blink_timer.setInterval(500)
        self._blink_timer.timeout.connect(self._blink_tick)

    # ── layout ────────────────────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(0)
        root.setContentsMargins(0, 0, 0, 0)

        # ── stat tile header row ──────────────────────────────────────────────
        hdr_container = QWidget()
        hdr_container.setStyleSheet(f"background:{BG_1}; border-bottom:1px solid {LINE};")
        hdr = QHBoxLayout(hdr_container)
        hdr.setContentsMargins(8, 6, 8, 6)
        hdr.setSpacing(6)

        # state tile (text, not numeric)
        state_tile = QWidget()
        state_tile.setStyleSheet(f"background:{BG_1}; border:1px solid {LINE};")
        state_inner = QVBoxLayout(state_tile)
        state_inner.setContentsMargins(10, 4, 10, 4)
        state_inner.setSpacing(2)
        cap_state = QLabel("STATE")
        cap_state.setAlignment(Qt.AlignCenter)
        cap_state.setStyleSheet(
            f"background:transparent; border:none; color:{DIM};"
            f"font-family:{FONT}; font-size:9px; letter-spacing:1px;"
        )
        self._hdr_state = QLabel("SETUP")
        self._hdr_state.setAlignment(Qt.AlignCenter)
        self._hdr_state.setStyleSheet(
            f"background:transparent; border:none;"
            f"font-family:{FONT}; font-size:16px; font-weight:bold; color:{DIM};"
        )
        state_inner.addWidget(cap_state)
        state_inner.addWidget(self._hdr_state)
        hdr.addWidget(state_tile, stretch=2)

        self._tile_tof = SegTile("TOF MM", digits=5)
        self._tile_tof.display(0)
        hdr.addWidget(self._tile_tof, stretch=1)

        self._tile_theta = SegTile("THETA DEG", digits=4)
        self._tile_theta.display(0)
        hdr.addWidget(self._tile_theta, stretch=1)

        self._hdr_pose = QLabel("x=---  y=---")
        self._hdr_pose.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        self._hdr_pose.setStyleSheet(
            f"color:{DIM}; font-family:{FONT}; font-size:11px; padding:0 10px;"
        )
        hdr.addWidget(self._hdr_pose, stretch=1)
        hdr.addStretch()

        root.addWidget(hdr_container)

        # ── body splitter ─────────────────────────────────────────────────────
        body_splitter = QSplitter(Qt.Horizontal)
        body_splitter.setHandleWidth(1)
        root.addWidget(body_splitter, stretch=1)

        left_widget = QWidget()
        left_widget.setStyleSheet(f"background:{BG_0};")
        left = QVBoxLayout(left_widget)
        left.setContentsMargins(6, 6, 6, 6)
        left.setSpacing(6)

        cams = QHBoxLayout()
        cams.setSpacing(6)
        self._phone_lbl = self._cam_label("OVERHEAD", "MJPEG — NO FEED WITHOUT HARDWARE")
        self._bot_lbl   = self._cam_label("BOT S3",   "WAITING FOR STREAM")
        cams.addWidget(self._phone_lbl)
        cams.addWidget(self._bot_lbl)
        left.addLayout(cams)
        left.addStretch()

        body_splitter.addWidget(left_widget)

        sidebar = self._build_sidebar()
        body_splitter.addWidget(sidebar)
        body_splitter.setSizes([960, 300])

        # ── bottom status bar ─────────────────────────────────────────────────
        bar_container = QWidget()
        bar_container.setFixedHeight(26)
        bar_container.setStyleSheet(
            f"background:{BG_1}; border-top:1px solid {LINE};"
        )
        bar = QHBoxLayout(bar_container)
        bar.setContentsMargins(10, 0, 10, 0)
        bar.setSpacing(0)

        legend = QLabel("START · STOP · QUEUE")
        legend.setStyleSheet(
            f"color:{DIM}; font-family:{FONT}; font-size:10px; border:none;"
        )
        bar.addWidget(legend)
        bar.addStretch()

        self._bar_telem = QLabel("SETUP · TOF:--- · θ:---°")
        self._bar_telem.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._bar_telem.setStyleSheet(
            f"color:{DIM}; font-family:{FONT}; font-size:10px; border:none;"
        )
        bar.addWidget(self._bar_telem)

        root.addWidget(bar_container)

    def _cam_label(self, caption: str, placeholder: str) -> QWidget:
        container = QWidget()
        container.setStyleSheet(
            f"background:{BG_1}; border:1px solid {LINE};"
        )
        vl = QVBoxLayout(container)
        vl.setContentsMargins(0, 0, 0, 0)
        vl.setSpacing(0)

        cap = QLabel(caption)
        cap.setFixedHeight(18)
        cap.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        cap.setStyleSheet(
            f"background:{BG_2}; color:{DIM}; border:none; border-bottom:1px solid {LINE};"
            f"font-family:{FONT}; font-size:9px; letter-spacing:1px; padding:0 6px;"
        )
        vl.addWidget(cap)

        feed = QLabel(placeholder, alignment=Qt.AlignCenter)
        feed.setFixedSize(480, 270)
        feed.setStyleSheet(
            f"background:{BG_1}; color:{DIM}; border:none; font-family:{FONT}; font-size:10px;"
        )
        feed.setWordWrap(True)
        vl.addWidget(feed)
        vl.addStretch()

        # attach the QLabel for frame updates
        if caption == "OVERHEAD":
            self._phone_lbl_feed = feed
        else:
            self._bot_lbl_feed = feed

        # keep old names pointing to the feed labels for frame slots
        if caption == "OVERHEAD":
            container._feed = feed
            self._phone_lbl = container
        else:
            container._feed = feed
            self._bot_lbl = container

        return container

    def _build_sidebar(self) -> QWidget:
        sb = QWidget()
        sb.setFixedWidth(260)
        sb.setStyleSheet(
            f"background:{BG_1}; border-left:1px solid {LINE};"
        )

        layout = QVBoxLayout(sb)
        layout.setSpacing(6)
        layout.setContentsMargins(10, 10, 10, 10)

        hdr_lbl = QLabel("MISSION CONTROL")
        hdr_lbl.setAlignment(Qt.AlignCenter)
        hdr_lbl.setStyleSheet(
            f"color:{DIM}; font-family:{FONT}; font-size:9px; letter-spacing:2px; border:none;"
        )
        layout.addWidget(hdr_lbl)

        self._sprite = RobotSprite()
        layout.addWidget(self._sprite, alignment=Qt.AlignHCenter)

        self._sb_state = QLabel("SETUP")
        self._sb_state.setAlignment(Qt.AlignCenter)
        self._sb_state.setWordWrap(True)
        self._sb_state.setStyleSheet(
            f"font-family:{FONT}; font-size:14px; font-weight:bold; color:{DIM};"
            f"padding:6px; border:1px solid {LINE}; background:{BG_2};"
        )
        layout.addWidget(self._sb_state)

        tof_row = QHBoxLayout()
        self._sb_tile_tof = SegTile("TOF MM", digits=5)
        self._sb_tile_tof.display(0)
        tof_row.addWidget(self._sb_tile_tof)
        layout.addLayout(tof_row)

        pose_row = QHBoxLayout()
        self._sb_tile_x     = SegTile("X M",   digits=4)
        self._sb_tile_y     = SegTile("Y M",   digits=4)
        self._sb_tile_theta = SegTile("θ DEG", digits=4)
        for t in (self._sb_tile_x, self._sb_tile_y, self._sb_tile_theta):
            t.display(0)
            pose_row.addWidget(t)
        layout.addLayout(pose_row)

        sep1 = QFrame()
        sep1.setFrameShape(QFrame.HLine)
        sep1.setStyleSheet(f"background:{LINE}; max-height:1px; border:none;")
        layout.addWidget(sep1)

        box_hdr = QLabel("BOX STATUS")
        box_hdr.setStyleSheet(
            f"color:{DIM}; font-family:{FONT}; font-size:9px; letter-spacing:1px; border:none;"
        )
        layout.addWidget(box_hdr)

        self._box_labels: dict = {}
        for bid in BOX_ARUCO_IDS:
            row = QHBoxLayout()
            row.setSpacing(6)
            id_lbl = QLabel(f"BOX {bid}")
            id_lbl.setFixedWidth(56)
            id_lbl.setStyleSheet(
                f"color:{DIM}; font-family:{FONT}; font-size:10px; border:none;"
            )
            st_lbl = QLabel("UNKNOWN")
            st_lbl.setAlignment(Qt.AlignCenter)
            st_lbl.setStyleSheet(
                BOX_STATUS_STYLES["UNKNOWN"]
                + f" font-family:{FONT}; font-size:10px; padding:2px 4px;"
            )
            self._box_labels[bid] = st_lbl
            row.addWidget(id_lbl)
            row.addWidget(st_lbl)
            layout.addLayout(row)

        layout.addStretch()

        sep2 = QFrame()
        sep2.setFrameShape(QFrame.HLine)
        sep2.setStyleSheet(f"background:{LINE}; max-height:1px; border:none;")
        layout.addWidget(sep2)

        self._queue_btn = QPushButton("MANAGE REPAIR QUEUE")
        self._queue_btn.setCursor(Qt.PointingHandCursor)
        self._queue_btn.setStyleSheet(
            _BTN_BASE + _BTN_DISABLED
        )
        self._queue_btn.setEnabled(False)
        self._queue_btn.clicked.connect(self._open_queue_dialog)
        layout.addWidget(self._queue_btn)

        self._start_btn = QPushButton("START RUN")
        self._start_btn.setCursor(Qt.PointingHandCursor)
        self._start_btn.setStyleSheet(_BTN_PRIMARY)
        self._start_btn.clicked.connect(self._on_start)
        layout.addWidget(self._start_btn)

        stop_btn = QPushButton("STOP BOT")
        stop_btn.setCursor(Qt.PointingHandCursor)
        stop_btn.setStyleSheet(_BTN_STOP)
        stop_btn.clicked.connect(self._on_stop)
        layout.addWidget(stop_btn)

        return sb

    # ── bridge ────────────────────────────────────────────────────────────────

    def _connect_bridge(self):
        self._bridge.sig_fsm_state.connect(self._on_fsm_state)
        self._bridge.sig_inspection.connect(self._on_inspection)
        self._bridge.sig_bot_pose.connect(self._on_bot_pose)
        self._bridge.sig_tof.connect(self._on_tof_reading)

    # ── slots ─────────────────────────────────────────────────────────────────

    def _on_phone_frame(self, img: QImage):
        pix = QPixmap.fromImage(img).scaled(
            480, 270, Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self._phone_lbl_feed.setPixmap(pix)

    def _on_bot_frame(self, img: QImage):
        pix = QPixmap.fromImage(img).scaled(
            480, 270, Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self._bot_lbl_feed.setPixmap(pix)

    def _on_fsm_state(self, state: str):
        self._fsm_state = state
        color = STATE_COLORS.get(state, INK)

        self._hdr_state.setText(state)
        self._hdr_state.setStyleSheet(
            f"background:transparent; border:none;"
            f"font-family:{FONT}; font-size:16px; font-weight:bold; color:{color};"
        )

        self._sb_state.setText(state)
        self._sb_state.setStyleSheet(
            f"font-family:{FONT}; font-size:14px; font-weight:bold; color:{color};"
            f"padding:6px; border:1px solid {LINE}; background:{BG_2};"
        )

        self._bar_telem.setText(
            f"{state} · TOF:{self._bar_tof_mm:.0f} · θ:{math.degrees(self._bar_theta):.0f}°"
        )
        self._bar_telem.setStyleSheet(
            f"color:{color if color != INK else DIM}; font-family:{FONT}; font-size:10px; border:none;"
        )

        if state in ("AWAITING_HUMAN", "REVIEWING_QUEUE"):
            self._blink_timer.start()
        else:
            self._blink_timer.stop()
            self._blink_on = True

        if state == "REVIEWING_QUEUE":
            self._queue_btn.setEnabled(True)
        else:
            self._queue_btn.setEnabled(False)

        if state not in ("SETUP", "MISSION_COMPLETE"):
            self._start_btn.setVisible(False)

        if state == "MISSION_COMPLETE":
            self._start_btn.setVisible(True)
            self._start_btn.setText("MISSION COMPLETE")
            self._start_btn.setStyleSheet(
                f"QPushButton {{ background:{BG_2}; color:{OK};"
                f"  font-family:{FONT}; font-size:13px; font-weight:bold;"
                f"  padding:8px; border:1px solid {OK}; border-radius:0px; }}"
            )
            self._start_btn.setEnabled(False)

    def _on_inspection(self, result: str):
        parts = result.split(":")
        if len(parts) < 2:
            return
        verdict = parts[0].upper()
        try:
            box_id = int(parts[1])
        except ValueError:
            return

        self._box_status[box_id] = verdict

        lbl = self._box_labels.get(box_id)
        if lbl:
            lbl.setText(verdict)
            lbl.setStyleSheet(
                BOX_STATUS_STYLES.get(verdict, BOX_STATUS_STYLES["UNKNOWN"])
                + f" font-family:{FONT}; font-size:10px; padding:2px 4px;"
            )

        if verdict == "FAULT" and self._repair_dialog is None:
            self._repair_dialog = RepairDialog(box_id, self._node, parent=self)
            self._repair_dialog.finished.connect(
                lambda _: setattr(self, "_repair_dialog", None)
            )
            self._repair_dialog.open()

    def _on_bot_pose(self, x: float, y: float, theta: float):
        self._bar_theta = theta
        deg = math.degrees(theta)
        self._hdr_pose.setText(f"x={x:.2f}  y={y:.2f}")
        self._tile_theta.display(round(deg))
        self._sb_tile_x.display(round(x, 2))
        self._sb_tile_y.display(round(y, 2))
        self._sb_tile_theta.display(round(deg))
        self._sprite.set_theta(theta)
        self._bar_telem.setText(
            f"{self._fsm_state} · TOF:{self._bar_tof_mm:.0f} · θ:{deg:.0f}°"
        )

    def _on_tof_reading(self, mm: float):
        self._bar_tof_mm = mm
        self._tile_tof.display(round(mm))
        self._sb_tile_tof.display(round(mm))
        self._bar_telem.setText(
            f"{self._fsm_state} · TOF:{mm:.0f} · θ:{math.degrees(self._bar_theta):.0f}°"
        )

    def _on_start(self):
        self._node.publish_decision("START")
        self._start_btn.setVisible(False)

    def _on_stop(self):
        self._node.publish_stop()

    def _open_queue_dialog(self):
        queued_boxes = [bid for bid, st in self._box_status.items() if st == "QUEUED"]
        self._queue_dialog = RepairQueueDialog(queued_boxes, self._node, parent=self)
        self._queue_dialog.finished.connect(
            lambda _: setattr(self, "_queue_dialog", None)
        )
        self._queue_dialog.open()

    def _blink_tick(self):
        self._blink_on = not self._blink_on
        state = self._fsm_state
        if state not in STATE_COLORS:
            return
        color = STATE_COLORS[state] if self._blink_on else BG_2
        self._sb_state.setStyleSheet(
            f"font-family:{FONT}; font-size:14px; font-weight:bold; color:{color};"
            f"padding:6px; border:1px solid {LINE}; background:{BG_2};"
        )

    def closeEvent(self, event):
        self._blink_timer.stop()
        if self._phone_cam is not None:
            self._phone_cam.stop()
        if hasattr(self, "_bot_cam") and self._bot_cam is not None:
            self._bot_cam.stop()
        event.accept()


# ── standalone entry point ────────────────────────────────────────────────────

def main(args=None):
    if not HAS_QT:
        print("PyQt5 not found — cannot run GUI")
        return

    rclpy.init(args=args)

    # QApplication must exist before any QObject (including RosBridge).
    app    = QApplication(sys.argv)
    bridge = RosBridge()
    node   = GuiNode(bridge)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    win = MainWindow(bridge, node)
    win.show()
    app.exec_()

    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
