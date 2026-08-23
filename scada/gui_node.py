"""
gui_node — PyQt5 SCADA dashboard (4-panel layout).
Run via: uv run scada/main.py  (or standalone: uv run scada/gui_node.py)

Panels:
  Top-left:  Phone cam feed (overhead MJPEG)
  Top-right: Bot cam feed (S3 MJPEG — active only during INSPECTING)
  Bottom:    Arena schematic map (box dots, bot dot + heading arrow)
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
        QSplitter, QListWidget, QListWidgetItem, QAbstractItemView
    )
    from PyQt5.QtCore import (
        Qt, QThread, pyqtSignal, QObject, QTimer, QCoreApplication,
    )
    from PyQt5.QtGui import (
        QImage, QPixmap, QColor, QPainter, QPen, QBrush, QFont,
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
     QBrush, QFont, QListWidget, QListWidgetItem, QAbstractItemView) = (_Stub,) * 24
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

STATE_COLORS = {
    "SETUP":            "#888888",
    "IDLE":             "#cccccc",
    "NAVIGATING":       "#4488ff",
    "DOCKING":          "#ffcc00",
    "INSPECTING":       "#ffcc00",
    "AWAITING_HUMAN":   "#ff8800",
    "REVIEWING_QUEUE":  "#0dcaf0",
    "REPAIRING":        "#ff8800",
    "RETURNING_HOME":   "#4488ff",
    "MISSION_COMPLETE": "#00cc88",
}

BOX_STATUS_STYLES = {
    "UNKNOWN":  "background:#444; color:#aaa;",
    "OK":       "background:#116611; color:#aaffaa;",
    "FAULT":    "background:#881111; color:#ffaaaa;",
    "QUEUED":   "background:#664400; color:#ffcc88;",
    "REPAIRED": "background:#006666; color:#aaffff;",
    "SKIPPED":  "background:#2a0a0a; color:#ff8888; border:1px dashed #cc2200;",
}

def _box_dot_colors():
    if not HAS_QT:
        return {}
    return {
        "UNKNOWN":  QColor(100, 100, 100),
        "OK":       QColor( 50, 180,  50),
        "FAULT":    QColor(210,  50,  50),
        "QUEUED":   QColor(200, 130,   0),
        "REPAIRED": QColor(  0, 180, 180),
        "SKIPPED":  QColor(160,  50,  50),
    }

BOX_DOT_COLORS = _box_dot_colors()


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

        if "--sim" in sys.argv:
            self.create_subscription(RosImage, "/overhead_camera/image_raw",
                                     self._on_overhead_img, 5)
            self.create_subscription(RosImage, "/camera/camera_sensor/image_raw",
                                     self._on_bot_cam_img, 5)
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
        img.fill(QColor(20, 20, 25))
        if HAS_QT:
            painter = QPainter(img)
            painter.setPen(QColor(150, 150, 150))
            font = QFont("Consolas", 14, QFont.Bold)
            painter.setFont(font)
            painter.drawText(img.rect(), Qt.AlignCenter, "CAMERA OFFLINE\nWAITING FOR SIGNAL...")
            painter.end()
        return img

    def stop(self):
        self._running = False
        self.wait(3000)


# ── arena schematic map ───────────────────────────────────────────────────────

class ArenaMap(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._box_status = {bid: "UNKNOWN" for bid in BOX_ARUCO_IDS}
        self._bot_pose   = None   # (x_m, y_m, theta_rad) or None
        self._bg_img     = None
        self.setMinimumSize(320, 280)

    def set_background_image(self, img: QImage):
        self._bg_img = img
        self.update()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background-color: #0d0d1a;")

    def set_box_status(self, box_id: int, status: str):
        self._box_status[box_id] = status
        self.update()

    def set_bot_pose(self, x: float, y: float, theta: float):
        self._bot_pose = (x, y, theta)
        self.update()

    def _to_widget(self, x_m: float, y_m: float, x0: int, y0: int, sq: int):
        arena_m = ARENA_SIZE_CM / 100.0
        wx = x0 + (x_m / arena_m) * sq
        wy = y0 + sq - (y_m / arena_m) * sq
        return int(wx), int(wy)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        margin = 32
        total_w = self.width()  - 2 * margin
        total_h = self.height() - 2 * margin
        sq = min(total_w, total_h)
        x0 = margin + (total_w - sq) // 2
        y0 = margin + (total_h - sq) // 2

        # arena background
        if self._bg_img is not None:
            inset = int((0.075 / 2.0) * sq)
            side  = sq - 2 * inset
            painter.drawImage(
                x0 + inset, y0 + inset,
                self._bg_img.scaled(side, side, Qt.IgnoreAspectRatio, Qt.SmoothTransformation),
            )
        else:
            painter.setPen(QPen(QColor(60, 60, 90), 2))
            painter.setBrush(QColor(15, 15, 25))
            painter.drawRect(x0, y0, sq, sq)

        painter.setOpacity(0.7)

        painter.setPen(QColor(120, 120, 150))
        painter.setFont(QFont("Consolas", 10, QFont.Bold))
        painter.drawText(x0 + 4, y0 - 6,
                         f"ARENA BLUEPRINT ({ARENA_SIZE_CM} × {ARENA_SIZE_CM} cm)")

        # faint grid every 50 cm
        painter.setPen(QPen(QColor(40, 50, 80), 1, Qt.DashLine))
        steps = int(ARENA_SIZE_CM / 50)
        for i in range(1, steps):
            frac = i / steps
            xt = x0 + int(frac * sq)
            yt = y0 + int(frac * sq)
            painter.drawLine(xt, y0, xt, y0 + sq)
            painter.drawLine(x0, yt, x0 + sq, yt)

        # box dots
        for box_id, (bx_m, by_m) in BOX_POSITIONS.items():
            status = self._box_status.get(box_id, "UNKNOWN")
            color  = BOX_DOT_COLORS.get(status, BOX_DOT_COLORS["UNKNOWN"])
            wx, wy = self._to_widget(bx_m, by_m, x0, y0, sq)

            painter.setBrush(QBrush(color))
            pen = (QPen(QColor(200, 50, 50), 2, Qt.DashLine)
                   if status == "SKIPPED"
                   else QPen(QColor(220, 220, 220), 1))
            painter.setPen(pen)
            painter.drawEllipse(wx - 6, wy - 6, 12, 12)

            painter.setPen(QColor(255, 255, 255))
            painter.setFont(QFont("Consolas", 10, QFont.Bold))
            painter.drawText(wx - 8, wy - 10, str(box_id))

        # bot dot + heading arrow
        if self._bot_pose is not None:
            bx, by, theta = self._bot_pose
            # Visually offset the dot forward by 6cm so it sits on the chassis center
            bx += 0.06 * math.cos(theta)
            by += 0.06 * math.sin(theta)
            
            wx, wy = self._to_widget(bx, by, x0, y0, sq)
            painter.setBrush(QBrush(QColor(0, 140, 255)))
            painter.setPen(QPen(QColor(255, 255, 255), 1))
            painter.drawEllipse(wx - 9, wy - 9, 18, 18)
            dx = int(24 * math.cos(theta))
            dy = int(-24 * math.sin(theta))
            painter.setPen(QPen(QColor(255, 220, 0), 2))
            painter.drawLine(wx, wy, wx + dx, wy + dy)

        painter.end()


# ── repair modal ──────────────────────────────────────────────────────────────

class RepairDialog(QDialog):
    def __init__(self, box_id: int, node: GuiNode, parent=None):
        super().__init__(parent)
        self._node   = node

        self.setWindowTitle("FAULT DETECTED")
        self.setModal(True)
        self.setMinimumWidth(360)
        self.setStyleSheet(
            "QDialog { background-color:#1e1e24; border: 2px solid #dc3545; border-radius: 8px; }"
            "QLabel { color:#ffffff; font-family:'Segoe UI', sans-serif; }"
        )

        layout = QVBoxLayout(self)
        layout.setSpacing(16)
        layout.setContentsMargins(24, 24, 24, 24)

        title = QLabel(f"FAULT DETECTED (Box {box_id})")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: #dc3545;"
            "padding: 10px; background-color: #2b1114; border-radius: 6px;"
        )
        layout.addWidget(title)

        for label, decision, style in [
            ("Repair Now",      "REPAIR_NOW",
             "QPushButton { background:#d97706; color:white; font-weight:bold; border-radius:6px; font-size:14px; padding:10px; } "
             "QPushButton:hover { background:#b45309; }"),
            ("Queue for Later", "QUEUE",
             "QPushButton { background:#4b5563; color:white; font-weight:bold; border-radius:6px; font-size:14px; padding:10px; } "
             "QPushButton:hover { background:#374151; }"),
            ("Skip",            "SKIP",
             "QPushButton { background:#1f2937; color:#9ca3af; font-weight:bold; border-radius:6px; font-size:14px; padding:10px; border:1px solid #374151;} "
             "QPushButton:hover { background:#111827; }"),
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
            "QDialog { background-color:#1e1e24; border: 2px solid #0dcaf0; border-radius: 8px; }"
            "QLabel { color:#ffffff; font-family:'Segoe UI', sans-serif; font-size: 14px; }"
        )
        
        layout = QVBoxLayout(self)
        layout.setSpacing(16)
        layout.setContentsMargins(24, 24, 24, 24)
        
        title = QLabel("MANAGE REPAIR QUEUE")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: #0dcaf0;"
            "padding: 10px; background-color: #0c2b33; border-radius: 6px;"
        )
        layout.addWidget(title)
        
        if not queued_boxes:
            lbl = QLabel("No boxes currently in queue.")
            lbl.setAlignment(Qt.AlignCenter)
            layout.addWidget(lbl)
            btn = QPushButton("Close")
            btn.setStyleSheet("QPushButton { background:#374151; color:white; border-radius:6px; font-size:14px; padding:10px; } QPushButton:hover { background:#1f2937; }")
            btn.clicked.connect(self.reject)
            layout.addWidget(btn)
            return
            
        lbl = QLabel("Click boxes below to add to execution order:")
        layout.addWidget(lbl)
        
        self._box_layout = QHBoxLayout()
        for bid in queued_boxes:
            btn = QPushButton(f"Box {bid}")
            btn.setStyleSheet("QPushButton { background:#374151; color:white; border-radius:4px; padding:10px; font-weight:bold; }")
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda checked, b=bid: self._toggle_box(b))
            self._box_layout.addWidget(btn)
        layout.addLayout(self._box_layout)
        
        self._order_lbl = QLabel("Selected Order: [None]")
        self._order_lbl.setStyleSheet("color:#0dcaf0; font-weight:bold;")
        layout.addWidget(self._order_lbl)
        
        btn_layout = QHBoxLayout()
        self._skip_btn = QPushButton("Skip All & Finish")
        self._skip_btn.setStyleSheet("QPushButton { background:#dc3545; color:white; font-weight:bold; border-radius:6px; font-size:14px; padding:10px; } QPushButton:hover { background:#b02a37; }")
        self._skip_btn.setCursor(Qt.PointingHandCursor)
        self._skip_btn.clicked.connect(self._finish)
        
        self._exec_btn = QPushButton("Execute Selected")
        self._exec_btn.setStyleSheet("QPushButton { background:#198754; color:white; font-weight:bold; border-radius:6px; font-size:14px; padding:10px; } QPushButton:hover { background:#157347; }")
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
            bid_str = btn.text().split("\n")[0].replace("Box ", "")
            try:
                bid = int(bid_str)
            except: continue
            
            if bid in self._selected_order:
                idx = self._selected_order.index(bid) + 1
                btn.setText(f"Box {bid}\n(#{idx})")
                btn.setStyleSheet("QPushButton { background:#198754; color:white; border-radius:4px; padding:10px; font-weight:bold; }")
            else:
                btn.setText(f"Box {bid}")
                btn.setStyleSheet("QPushButton { background:#374151; color:white; border-radius:4px; padding:10px; font-weight:bold; }")
                
        if self._selected_order:
            self._order_lbl.setText("Selected Order: " + ", ".join(f"Box {b}" for b in self._selected_order))
        else:
            self._order_lbl.setText("Selected Order: [None]")
            
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

        self.setWindowTitle("Warden SCADA")
        self.resize(1300, 820)
        self.setStyleSheet(
            "background-color:#0b0c10; color:#c5c6c7; font-family:'Segoe UI', Consolas, sans-serif;"
        )

        self._build_ui()
        self._connect_bridge()

        self._phone_cam = None
        if "--sim" in sys.argv:
            bridge.sig_phone_frame.connect(self._on_phone_frame)
            bridge.sig_warp_frame.connect(self._on_warp_frame)
        else:
            self._phone_cam = CameraThread(PHONE_STREAM_URL, fps=20)
            self._phone_cam.frame_ready.connect(self._on_phone_frame)
            self._phone_cam.start()

        self._bot_cam_started = "--sim" in sys.argv
        self._bot_cam = None
        if "--sim" in sys.argv:
            bridge.sig_bot_frame.connect(self._on_bot_frame)
            bridge.sig_tof_frame.connect(self._on_tof_frame)
        else:
            self._bot_cam = CameraThread(S3_STREAM_URL, fps=15)
            self._bot_cam.frame_ready.connect(self._on_bot_frame)

        self._blink_timer = QTimer()
        self._blink_timer.setInterval(500)
        self._blink_timer.timeout.connect(self._blink_tick)

    # ── layout ────────────────────────────────────────────────────────────────

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(4)
        root.setContentsMargins(6, 6, 6, 6)

        # header
        hdr = QHBoxLayout()
        self._hdr_state = QLabel("STATE: SETUP")
        self._hdr_state.setStyleSheet(
            f"font-size:15px; font-weight:bold;"
            f"color:{STATE_COLORS['SETUP']}; padding:3px 10px;"
        )
        self._hdr_tof  = QLabel("TOF: --- mm")
        self._hdr_tof.setStyleSheet("font-size:12px; color:#666; padding:3px 8px;")
        self._hdr_pose = QLabel("Bot: x=--- y=--- θ=---")
        self._hdr_pose.setStyleSheet("font-size:12px; color:#666; padding:3px 8px;")
        hdr.addWidget(self._hdr_state)
        hdr.addWidget(self._hdr_tof)
        hdr.addWidget(self._hdr_pose)
        hdr.addStretch()
        root.addLayout(hdr)

        sep = QFrame(); sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color:#1e1e2e;"); root.addWidget(sep)

        body_splitter = QSplitter(Qt.Horizontal)
        root.addWidget(body_splitter, stretch=1)

        left_widget = QWidget()
        left = QVBoxLayout(left_widget)
        left.setContentsMargins(0, 0, 0, 0)

        cams = QHBoxLayout()
        self._phone_lbl = self._cam_label("Phone Cam — overhead MJPEG\n(no feed without hardware)")
        self._bot_lbl   = self._cam_label("Bot Cam (S3)\n(activates during INSPECTING)")
        cams.addWidget(self._phone_lbl)
        cams.addWidget(self._bot_lbl)
        left.addLayout(cams)

        bottom_row = QHBoxLayout()
        self._arena = ArenaMap()
        bottom_row.addWidget(self._arena, stretch=1)
        
        self._tof_lbl = self._cam_label("TOF Top-Down\n(activates during --sim)")
        self._tof_lbl.setStyleSheet("background:#333333; color:#aaa; border:1px solid #1e1e2e; font-size:10px;")
        bottom_row.addWidget(self._tof_lbl)
        
        left.addLayout(bottom_row, stretch=1)

        body_splitter.addWidget(left_widget)
        
        sidebar = self._build_sidebar()
        body_splitter.addWidget(sidebar)
        
        body_splitter.setSizes([900, 300])

    def _cam_label(self, text: str) -> QLabel:
        lbl = QLabel(text, alignment=Qt.AlignCenter)
        lbl.setFixedSize(480, 270)
        lbl.setStyleSheet(
            "background:#111; color:#444; border:1px solid #1e1e2e; font-size:10px;"
        )
        return lbl

    def _build_sidebar(self) -> QWidget:
        sb = QWidget()
        sb.setFixedWidth(248)
        sb.setStyleSheet("background:#090914; border-left:1px solid #1e1e2e;")

        layout = QVBoxLayout(sb)
        layout.setSpacing(6)
        layout.setContentsMargins(10, 10, 10, 10)

        hdr = QLabel("MISSION CONTROL")
        hdr.setAlignment(Qt.AlignCenter)
        hdr.setStyleSheet("color:#555; font-size:11px; letter-spacing:2px;")
        layout.addWidget(hdr)

        self._sb_state = QLabel("SETUP")
        self._sb_state.setAlignment(Qt.AlignCenter)
        self._sb_state.setWordWrap(True)
        self._sb_state.setStyleSheet(
            f"font-size:17px; font-weight:bold; color:{STATE_COLORS['SETUP']};"
            "padding:8px; border:1px solid #222; border-radius: 6px;"
        )
        layout.addWidget(self._sb_state)

        self._sb_tof  = QLabel("TOF: --- mm", alignment=Qt.AlignCenter)
        self._sb_pose = QLabel("x=---   y=---\nθ=---°",  alignment=Qt.AlignCenter)
        for w in (self._sb_tof, self._sb_pose):
            w.setStyleSheet("color:#666; font-size:11px;")
            layout.addWidget(w)

        sep1 = QFrame(); sep1.setFrameShape(QFrame.HLine)
        sep1.setStyleSheet("color:#222;"); layout.addWidget(sep1)

        layout.addWidget(QLabel("Box Status:"))
        self._box_labels: dict = {}
        for bid in BOX_ARUCO_IDS:
            row = QHBoxLayout()
            id_lbl = QLabel(f"Box {bid}")
            id_lbl.setFixedWidth(52)
            id_lbl.setStyleSheet("color:#555; font-size:11px;")
            st_lbl = QLabel("UNKNOWN")
            st_lbl.setAlignment(Qt.AlignCenter)
            st_lbl.setStyleSheet(
                BOX_STATUS_STYLES["UNKNOWN"] + " font-size:11px; padding:2px 4px;"
            )
            self._box_labels[bid] = st_lbl
            row.addWidget(id_lbl)
            row.addWidget(st_lbl)
            layout.addLayout(row)

        layout.addStretch()

        sep2 = QFrame(); sep2.setFrameShape(QFrame.HLine)
        sep2.setStyleSheet("color:#222;"); layout.addWidget(sep2)

        self._queue_btn = QPushButton("MANAGE REPAIR QUEUE")
        self._queue_btn.setCursor(Qt.PointingHandCursor)
        self._queue_btn.setStyleSheet(
            """
            QPushButton {
                background-color: #0dcaf0; color: #000000;
                font-size: 13px; font-weight: bold; padding: 10px;
                border: 1px solid #0aa2c0; border-radius: 6px;
            }
            QPushButton:hover { background-color: #31d2f2; }
            QPushButton:disabled { background-color: #1a2a30; color: #444; border: 1px solid #222; }
            """
        )
        self._queue_btn.setEnabled(False)
        self._queue_btn.clicked.connect(self._open_queue_dialog)
        layout.addWidget(self._queue_btn)

        self._start_btn = QPushButton("START RUN")
        self._start_btn.setCursor(Qt.PointingHandCursor)
        self._start_btn.setStyleSheet(
            """
            QPushButton {
                background-color: #198754; color: white;
                font-size: 14px; font-weight: bold; padding: 10px;
                border: 1px solid #146c43; border-radius: 6px;
            }
            QPushButton:hover { background-color: #157347; }
            """
        )
        self._start_btn.clicked.connect(self._on_start)
        layout.addWidget(self._start_btn)

        stop_btn = QPushButton("STOP BOT")
        stop_btn.setCursor(Qt.PointingHandCursor)
        stop_btn.setStyleSheet(
            """
            QPushButton {
                background-color: #dc3545; color: white;
                font-size: 13px; font-weight: bold; padding: 8px;
                border: 1px solid #b02a37; border-radius: 6px;
            }
            QPushButton:hover { background-color: #bb2d3b; }
            """
        )
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
        self._phone_lbl.setPixmap(pix)

    def _on_warp_frame(self, img: QImage):
        self._arena.set_background_image(img)

    def _on_bot_frame(self, img: QImage):
        if "--sim" in sys.argv or self._fsm_state == "INSPECTING":
            pix = QPixmap.fromImage(img).scaled(
                480, 270, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            self._bot_lbl.setPixmap(pix)

    def _on_tof_frame(self, img: QImage):
        pix = QPixmap.fromImage(img).scaled(
            480, 270, Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self._tof_lbl.setPixmap(pix)

    def _on_fsm_state(self, state: str):
        self._fsm_state = state
        color = STATE_COLORS.get(state, "#cccccc")

        for lbl, prefix, font in [
            (self._hdr_state, "STATE: ",
             "font-size:15px; font-weight:bold; padding:3px 10px;"),
            (self._sb_state,  "",
             "font-size:17px; font-weight:bold; padding:8px; border:1px solid #222;"),
        ]:
            lbl.setText(prefix + state)
            lbl.setStyleSheet(f"color:{color}; {font}")

        if state in ("AWAITING_HUMAN", "REVIEWING_QUEUE"):
            self._blink_timer.start()
        else:
            self._blink_timer.stop()
            self._blink_on = True

        if state == "REVIEWING_QUEUE":
            self._queue_btn.setEnabled(True)
        else:
            self._queue_btn.setEnabled(False)

        if state == "INSPECTING" and not self._bot_cam_started and self._bot_cam is not None:
            self._bot_cam.start()
            self._bot_cam_started = True

        if state not in ("SETUP", "MISSION_COMPLETE"):
            self._start_btn.setVisible(False)

        if state == "MISSION_COMPLETE":
            self._start_btn.setVisible(True)
            self._start_btn.setText("MISSION COMPLETE")
            self._start_btn.setStyleSheet(
                "background:#0f5132; color:#d1e7dd;"
                "font-size:13px; font-weight:bold; padding:8px; border:1px solid #badbcc; border-radius: 6px;"
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

        self._arena.set_box_status(box_id, verdict)
        lbl = self._box_labels.get(box_id)
        if lbl:
            lbl.setText(verdict)
            lbl.setStyleSheet(
                BOX_STATUS_STYLES.get(verdict, BOX_STATUS_STYLES["UNKNOWN"])
                + " font-size:11px; padding:2px 4px;"
            )

        if verdict == "FAULT" and self._repair_dialog is None:
            self._repair_dialog = RepairDialog(box_id, self._node, parent=self)
            self._repair_dialog.finished.connect(
                lambda _: setattr(self, "_repair_dialog", None)
            )
            self._repair_dialog.open()

    def _on_bot_pose(self, x: float, y: float, theta: float):
        deg = math.degrees(theta)
        self._hdr_pose.setText(f"Bot: x={x:.2f} y={y:.2f} θ={deg:.0f}°")
        self._sb_pose.setText(f"x={x:.2f}   y={y:.2f}\nθ={deg:.0f}°")
        self._arena.set_bot_pose(x, y, theta)

    def _on_tof_reading(self, mm: float):
        txt = f"TOF: {mm:.0f} mm"
        self._hdr_tof.setText(txt)
        self._sb_tof.setText(txt)

    def _on_start(self):
        self._node.publish_decision("START")
        self._start_btn.setVisible(False)

    def _on_stop(self):
        self._node.publish_stop()

    def _open_queue_dialog(self):
        queued_boxes = [bid for bid, st in self._arena._box_status.items() if st == "QUEUED"]
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
        color = STATE_COLORS[state] if self._blink_on else "#333333"
        self._sb_state.setStyleSheet(
            f"color:{color}; font-size:17px; font-weight:bold;"
            "padding:8px; border:1px solid #222; border-radius: 6px;"
        )

    def closeEvent(self, event):
        self._blink_timer.stop()
        if self._phone_cam is not None:
            self._phone_cam.stop()
        if self._bot_cam is not None:
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
