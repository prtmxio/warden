"""
control_node — mission FSM + Nav2 action client.

States (in order):
  SETUP           → operator configures box positions in GUI
  IDLE            → waiting for START RUN
  NAVIGATING      → Nav2 driving to the pre-dock standoff south of the box
  ALIGNING        → rotate in place to perpendicular (face the box, ±PERP_TOL_DEG)
  APPROACH_SCAN   → TOF-guided straight creep to SCAN_STOP_MM, then read the ArUco
  INSPECTING      → resolve verdict (gated on square-on, full-QR view)
  AWAITING_HUMAN  → fault detected, waiting for operator decision
  APPROACH_DOCK   → TOF-guided straight creep to DOCK_STOP_MM before repairing
  REPAIRING       → bot executing physical repair sequence
  RETURNING_HOME  → Nav2 driving back to start pose
  MISSION_COMPLETE

Subscribes:
  /tof_range            Range
  /bot_status           String   (STATUS:REPAIRED, …)
  /operator_decision    String   (REPAIR_NOW / QUEUE / SKIP)
  /odom                 Odometry (closed-loop align/approach heading, odom≡map frame)
  /box_aruco_detected   String   (ArUco ID 10-13 from bot_cam_node)

Publishes:
  /bot_command        String   (SCAN / REPAIR_MODE / STOP)
  /inspection_result  String   (FAULT:<id> / OK:<id>)

Watchdog: publishes STOP to /bot_command every 400 ms while in AWAITING_HUMAN.
"""

import sys
import math
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Range
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray, PoseWithCovarianceStamped

try:
    from nav2_msgs.action import NavigateToPose
    HAS_NAV2 = True
except ImportError:
    HAS_NAV2 = False

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import (
    BOX_ARUCO_IDS, BOX_POSITIONS, fault_config,
    SCAN_STOP_MM, DOCK_STOP_MM, NAV_STANDOFF_M,
    PERP_TOL_DEG, ALIGN_MAX_PULSES, ALIGN_MAX_ROUNDS, ALIGN_SETTLE_TICKS,
    LAT_TOL_PX, LAT_TURN_TOL_DEG,
    LATERAL_TOL_M, APPROACH_SPEED, MOCK_REPAIR_CM,
    DOCK_NEAR_BAND_MM, DOCK_CREEP_SPEED, DOCK_ANTICIPATE_MM,
    DOCK_SETTLE, INSPECT_SWEEP, INSPECT_SWEEP_SPEED, INSPECT_SWEEP_AMPL,
    INSPECT_SWEEP_PAUSE_TICKS, INSPECT_SWEEP_MAX_PULSES,
    INSPECT_SWEEP_CYCLES, INSPECT_SWEEP_WP_TOL_DEG,
    INSPECT_STREAM_STALE_S, INSPECT_SAFETY_S,
    INSPECT_BACKOFF, INSPECT_BACKOFF_STEP_MM, INSPECT_BACKOFF_MAX_MM,
    INSPECT_BACKOFF_SETTLE_TICKS, INSPECT_BACKOFF_HOLD_TICKS,
    INSPECT_PRECAPTURE, INSPECT_PRECAPTURE_TTL_S, INSPECT_PRECAPTURE_DELAY_S,
    APPROACH_YAW_KP, ALIGN_YAW_KP,
    CAM_FWD, CAM_LAT, AXLE_FWD, AXLE_LAT,
    WATCHDOG_MS,
    ARENA_FREE, USE_TOF, PX_TURN_SIGN, PX_TURN_FIRST_DEG, PX_TURN_KP,
    PX_DRIVE_SPEED, PX_NEAR_GAP, PX_AIM_TOL_DEG, PX_APPROACH_STANDOFF,
    ODOM_FUSION, ODOM_STALE_S, FUSE_MIN_TRANS_M, FUSE_MIN_ROT_RAD,
    FUSE_FORGET, FUSE_MIN_DET, FUSE_MIN_SAMPLES, PX_ANG_MAX,
    SNAPSHOT_WINDOW, SNAPSHOT_MIN_SEEN,
    MOVE_PRIMITIVES, TURN_TOL_DEG, HEADING_SAMPLES, LAT_SAMPLES, SETTLE_S,
    PRIM_TIMEOUT_S, LAT_PX_PER_MM, LAT_GAIN, DOCK_STEP_MM, MOCK_REPAIR_MM, REPAIR_CONTACT_MM,
)

WATCHDOG_INTERVAL = WATCHDOG_MS / 1000.0 * 0.8   # send STOP slightly faster than 500 ms

# Boxes line the north wall and are always approached from the south, so the perpendicular
# (full-view) heading is a constant: face +Y.
TARGET_YAW = math.pi / 2.0

# Direct camera-guided drive-to-pre-dock (replaces Nav2 in the small open arena).
GOAL_TOL_M        = 0.08    # m  — close enough to the pre-dock point → start aligning
DRIVE_TURN_FIRST  = 20.0    # deg — if bearing error exceeds this, turn in place before driving
DRIVE_TURN_KP     = 1.2     # P gain for heading control during the drive
NAV_TOF_SAFETY_MM = 150.0   # mm — hard TOF floor during the drive; never push closer than this


def _wrap(a: float) -> float:
    """Wrap an angle to (-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def _circ_median(angles):
    """Robust central angle of a list (rad): component-wise median of the unit vectors, then
    atan2. Median (not mean) so an occasional flipped/garbage ArUco heading can't drag the result."""
    if not angles:
        return None
    xs = sorted(math.cos(a) for a in angles)
    ys = sorted(math.sin(a) for a in angles)
    n = len(xs)
    mx = xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])
    my = ys[n // 2] if n % 2 else 0.5 * (ys[n // 2 - 1] + ys[n // 2])
    return math.atan2(my, mx)


def _median(vals):
    """Plain scalar median of a list, or None if empty."""
    if not vals:
        return None
    s = sorted(vals)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _camera_x(x: float, yaw: float) -> float:
    """World-x of the S3 camera for a base pose (x, yaw). base_footprint is off the bot
    centerline, so this — not base_x — is what must line up with the box centre."""
    return x + CAM_FWD * math.cos(yaw) - CAM_LAT * math.sin(yaw)


def _axle_x(x: float, yaw: float) -> float:
    """World-x of the wheel-axle midpoint (the in-place rotation centre). Centering this on the
    box is rotation-invariant, so the perp-turn doesn't move it — and since the camera is on the
    same centerline, the camera lands centred once perpendicular."""
    return x + AXLE_FWD * math.cos(yaw) - AXLE_LAT * math.sin(yaw)


class FSM:
    SETUP           = "SETUP"
    IDLE            = "IDLE"
    NAVIGATING      = "NAVIGATING"
    ALIGNING        = "ALIGNING"        # rotate in place to perpendicular
    APPROACH_SCAN   = "APPROACH_SCAN"   # TOF creep to scan distance, then read ArUco
    APPROACH_DOCK   = "APPROACH_DOCK"   # TOF creep to dock distance, then repair
    INSPECTING      = "INSPECTING"
    AWAITING_HUMAN  = "AWAITING_HUMAN"
    REVIEWING_QUEUE = "REVIEWING_QUEUE"
    REPAIRING       = "REPAIRING"
    RETURNING_HOME  = "RETURNING_HOME"
    MISSION_COMPLETE = "MISSION_COMPLETE"


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # --- publishers ---
        self._cmd_pub    = self.create_publisher(String, "/bot_command",      10)
        self._result_pub = self.create_publisher(String, "/inspection_result", 10)
        self._state_pub  = self.create_publisher(String, "/fsm_state",         10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # --- subscribers ---
        self.create_subscription(Range,    "/tof_range",           self._on_tof,          10)
        self.create_subscription(String,   "/bot_status",          self._on_status,       10)
        self.create_subscription(String,   "/operator_decision",   self._on_decision,     10)
        self.create_subscription(String,   "/box_aruco_detected",  self._on_aruco_detected, 10)
        self.create_subscription(String,   "/bot_cam_status",      self._on_bot_cam_status, 10)
        # Closed-loop drive/align/approach steers by the OVERHEAD CAMERA pose (/robot_pose),
        # which lives in the same map frame as the detected box positions. Raw wheel /odom is a
        # different, drifting frame and must NOT be mixed with map-frame box coordinates.
        self.create_subscription(PoseWithCovarianceStamped, "/robot_pose", self._on_pose, 10)
        # /box_positions is latched by vision_node; match its QoS so we get the last sample.
        box_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseArray, "/box_positions", self._on_box_positions, box_qos)

        # Arena-free (pixel-space) inputs — bot marker + box-top markers in raw image pixels.
        self.create_subscription(PoseWithCovarianceStamped, "/bot_px", self._on_bot_px, 10)
        self.create_subscription(PoseArray, "/box_px", self._on_box_px, 10)
        # Wheel odometry — fused with the ~1 Hz camera to dead-reckon the pixel pose at 20 Hz.
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        # Nav2 action client (only wired up from Day 7 onward)
        if HAS_NAV2:
            self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # --- state ---
        self._state          = FSM.SETUP
        self._current_box_id = None         # aruco ID of the current box (None until inspected)
        self._current_box_pos = None        # (x, y) metres of the current target blob
        self._current_box_idx = 0           # 1-based visit counter, for logging/results
        self._detected_boxes = []           # latest (x, y) blobs from vision /box_positions
        self._visit_queue    = []            # list of (x, y) positions left to visit
        self._repair_queue   = []            # list of (pos, aruco_id) queued for later repair
        self._is_repair_phase = False        # true when executing queued repairs
        self._current_repair_batch = []      # list of (pos, aruco_id) to repair this batch
        self._tof_mm         = 9999.0
        self._box_results    = {}            # aruco_id → "FAULT"/"OK"/"REPAIRED"/"SKIP"
        self._pose           = None          # (x, y, yaw) from camera /robot_pose, map frame
        self._bot_px         = None          # (u, v, theta_img) from /bot_px — arena-free
        self._px_seq         = 0             # increments on every fresh /bot_px frame (~1 Hz)
        self._align_acted_seq = -1           # last frame the aligner issued a pulse on
        self._align_pulse_ticks = 0          # ticks left in the current align pulse
        self._align_pulse_cmd = 0.0          # angular.z of the current align pulse
        self._drive_acted_seq = -1           # last frame the homing turn-in-place pulsed on
        self._drive_pulse_ticks = 0          # ticks left in the current homing turn pulse
        self._drive_pulse_cmd = 0.0          # angular.z of the current homing turn pulse
        # Odom fusion: anchor the pixel pose to the camera each frame and dead-reckon on /odom.
        self._odom           = None          # latest (x, y, yaw, stamp) from /odom
        self._anchor_odom    = None          # odom pose captured at the last camera frame
        self._anchor_px      = None          # (u, v, theta) camera pose at the last camera frame
        self._fuse_A         = None          # 2x2 map odom-xy(m) → image-uv(px): (a11,a12,a21,a22)
        self._fuse_spp       = [0.0, 0.0, 0.0]   # running Σ odom·odomᵀ (sxx, sxy, syy)
        self._fuse_sqp       = [0.0, 0.0, 0.0, 0.0]  # running Σ cam·odomᵀ (b11, b12, b21, b22)
        self._fuse_nsamp     = 0             # position-calibration samples accumulated
        self._fuse_hdg_corr  = 0.0           # running Σ Δθ_img·Δyaw_odom → sign of heading mapping
        self._boxes_px       = []            # [(id, u, v, hdg_rad), …] box-top markers — arena-free
        self._beacon_cache   = {}            # bid → (u, v, hdg_rad, timestamp)
        self._beacon_history = deque(maxlen=SNAPSHOT_WINDOW)  # rolling set of seen IDs per frame
        self._current_beacon = None          # top-marker ArUco id being homed on — arena-free
        self._goal_xy        = None          # (x, y) pre-dock target for the direct drive
        self._drive_timer    = None          # active only during NAVIGATING (go-to-goal)
        self._align_timer    = None          # active only during ALIGNING
        self._align_phase    = "center_turn" # center_turn → center_drive → perp_turn
        self._align_ok_count = 0             # consecutive in-tolerance perp ticks
        self._align_pulse_count = 0          # pulses fired in the current rotation stage (safety cap)
        self._align_settle_ticks = 0         # motors-off settle remaining after the last pulse
        self._align_stage = "heading"        # staged aligner: heading → lat_check → lat_turn → lat_drive
        self._align_round = 0                # heading+lateral re-centre rounds done (safety cap)
        self._align_blind = 0                # consecutive align windows with no readable heading
        self._lat_sign = 1.0                 # sign of the lateral offset currently being corrected
        self._inspect_retries = 0            # failed full-view reads for current box
        self._inspect_timer  = None          # 5 s timeout waiting for ArUco from bot_cam
        self._precaptured_aruco = None       # box ArUco read early (ALIGNING/APPROACH_SCAN), replayed at INSPECTING
        self._precapture_ts  = 0.0           # monotonic time of that read (freshness gate)
        self._precapture_timer = None        # one-shot "scanning" hold before firing the replayed verdict
        self._repair_timer   = None          # drives the mock-repair nudge sequence
        self._mock_phase     = None          # "forward" → "pause" → "reverse" during mock repair
        self._mock_ticks     = 0             # ticks remaining in the current mock-repair phase
        self._dock_settling  = False         # backing off to a guaranteed ~10 cm gap after the stop
        self._sweep_timer    = None          # drives the inspection search sweep
        self._sweep_center_hdg = None        # overhead heading the bot docked at (sweep anchor)
        self._sweep_idx      = 0             # index into the scripted arc waypoints
        self._sweep_cycles   = 0             # completed full +arc→-arc→center passes
        self._sweep_cooldown = 0             # ticks of motors-off pause remaining
        self._sweep_stream_dead = False      # True if a pulse was ever skipped for a stale stream
        self._backoff_timer  = None          # drives the inspection distance back-off
        self._backoff_target = 0.0           # mm — TOF distance the current step is reversing toward
        self._backoff_settle = 0             # ticks of motors-off settle remaining this step
        self._backoff_hold   = 0             # ticks of end-of-band grace: keep looking before UNKNOWN
        self._backoff_stream_dead = False    # True if a step was ever held for a stale stream
        self._backoff_stale_logged = False
        self._last_cam_alive = 0.0           # monotonic time of the last /bot_cam_status heartbeat

        # --- encoder motion primitives (MOVE_PRIMITIVES) ---
        self._prim_pending = False           # a TURN/FWD is executing on the firmware; suppress cmd_vel
        self._prim_kind    = None            # "TURN" | "FWD" of the in-flight primitive
        self._prim_done_cb = None            # continuation to run on STATUS:*_DONE (or timeout)
        self._prim_deadline = 0.0            # monotonic deadline; past it, treat the primitive as done
        self._samp_timer   = None            # drives stationary measurement sampling
        self._samp         = None            # active sampler state dict
        self._approach_timer = None          # drives the primitive TOF approach
        self._approach_target = 0.0          # mm — TOF distance this approach is creeping toward
        self._approach_settling = False      # backing off to the readable dock gap
        self._approach_fwd_mm = 0.0          # mm driven forward this approach (safety cap)
        self._backoff_done_mm = 0.0          # mm backed off so far during inspect back-off
        self._mock_stage   = None            # "forward" | "reverse" during primitive mock repair
        self._repair_fwd_mm = 0.0            # mm the repair nosed forward (reverse mirrors it exactly)
        self._repair_remaining = 0.0         # mm left in the current stepped nose-in / back-out

        # watchdog timer fires every 400 ms; only sends STOP during AWAITING_HUMAN
        self._watchdog_timer = self.create_timer(WATCHDOG_INTERVAL, self._watchdog_tick)

        self._state = FSM.SETUP
        self._state_pub.publish(String(data=FSM.SETUP))
        self.get_logger().info("FSM: SETUP — waiting for operator START")

    # ------------------------------------------------------------------ FSM
    def _transition(self, new_state: str):
        old = self._state
        self._state = new_state
        ts = time.strftime("%H:%M:%S")
        self.get_logger().info(f"[{ts}] FSM: {old} → {new_state}")
        self._state_pub.publish(String(data=new_state))

        if new_state == FSM.IDLE:
            self._maybe_start_next_box()

        elif new_state == FSM.NAVIGATING:
            # New box visit — drop any ArUco cached for the previous box so it can never leak forward.
            self._precaptured_aruco = None
            self._cancel_precapture_timer()
            self._navigate_to_current_box()

        elif new_state == FSM.ALIGNING:
            self._start_align()

        elif new_state == FSM.APPROACH_SCAN:
            if MOVE_PRIMITIVES and ARENA_FREE:
                self._start_approach_prim(SCAN_STOP_MM)
            else:
                self.get_logger().info(f"Approach-scan: TOF creep to {SCAN_STOP_MM:.0f} mm")

        elif new_state == FSM.APPROACH_DOCK:
            self._dock_settling = False
            if not USE_TOF:
                # No usable TOF to creep the last few cm — repair from the scan position.
                self.get_logger().info("No TOF — repairing from scan distance")
                self._transition(FSM.REPAIRING)
            elif MOVE_PRIMITIVES and ARENA_FREE:
                self._start_approach_prim(DOCK_STOP_MM)
            else:
                self.get_logger().info(f"Approach-dock: TOF creep to {DOCK_STOP_MM:.0f} mm")

        elif new_state == FSM.INSPECTING:
            self._start_inspection()
            cached = self._take_precapture()
            if cached is not None:
                # Already read this box's marker on the way in — replay it after a short hold so it
                # reads as a scan at the stop, and skip the jerky back-off entirely.
                self.get_logger().info(
                    f"Replaying pre-captured ArUco {cached} after {INSPECT_PRECAPTURE_DELAY_S:.1f}s hold"
                )
                self._precapture_timer = self.create_timer(
                    INSPECT_PRECAPTURE_DELAY_S, lambda: self._fire_precapture(cached))
            elif ARENA_FREE and INSPECT_BACKOFF:
                if MOVE_PRIMITIVES:
                    self._start_inspect_backoff_prim()
                else:
                    self._start_inspect_backoff()
            elif ARENA_FREE and INSPECT_SWEEP:
                self._start_inspect_sweep()

        elif new_state == FSM.REPAIRING:
            if MOVE_PRIMITIVES and ARENA_FREE:
                self._start_mock_repair_prim()
            else:
                self._start_mock_repair()

        elif new_state == FSM.MISSION_COMPLETE:
            self.get_logger().info("=== MISSION COMPLETE ===")
            self.get_logger().info(f"Results: {self._box_results}")

    def _on_box_positions(self, msg: PoseArray):
        """Latest box blob centroids (world metres) from vision_node."""
        self._detected_boxes = [(p.position.x, p.position.y) for p in msg.poses]

    def _snapshot_visit_queue(self):
        """Lock in the boxes to patrol at START, ordered nearest-first from the bot.
        Falls back to arena_config.BOX_POSITIONS if vision hasn't detected any blobs
        yet (e.g. corners not all visible) so the mission can still run."""
        if ARENA_FREE:
            # Only admit beacons seen in >= SNAPSHOT_MIN_SEEN of the last SNAPSHOT_WINDOW frames.
            # A single-frame ghost (front marker glimpsed from above, reflection, etc.) never
            # reaches the patrol queue; a real box top seen across several frames always does.
            seen_counts = {}
            for frame_set in self._beacon_history:
                for bid in frame_set:
                    seen_counts[bid] = seen_counts.get(bid, 0) + 1
            stable = {bid for bid, cnt in seen_counts.items() if cnt >= SNAPSHOT_MIN_SEEN}
            beacons = [b for b in self._boxes_px if b[0] in stable]
            if not beacons:
                self.get_logger().warn(
                    f"No stable box-top markers (need {SNAPSHOT_MIN_SEEN}/{SNAPSHOT_WINDOW} frames) "
                    f"— seen counts: {seen_counts}"
                )
            start = self._bot_px[:2] if self._bot_px is not None else (0.0, 0.0)
            ordered, remaining = [], beacons
            cur = start
            while remaining:
                nxt = min(remaining, key=lambda b: math.hypot(b[1] - cur[0], b[2] - cur[1]))
                ordered.append(nxt)
                remaining.remove(nxt)
                cur = (nxt[1], nxt[2])
            self._visit_queue = [bid for bid, _, _, _ in ordered]
            self._current_box_idx = 0
            self.get_logger().info(
                f"Patrol plan: {len(ordered)} box(es) — beacon IDs "
                + " → ".join(str(bid) for bid in self._visit_queue)
            )
            return

        boxes = list(self._detected_boxes)
        if not boxes:
            boxes = list(BOX_POSITIONS.values())
            self.get_logger().warn(
                "No boxes detected from vision — falling back to BOX_POSITIONS"
            )
        start = self._pose[:2] if self._pose is not None else (0.0, 0.0)
        ordered, remaining = [], boxes
        cur = start
        while remaining:
            nxt = min(remaining, key=lambda b: math.hypot(b[0] - cur[0], b[1] - cur[1]))
            ordered.append(nxt)
            remaining.remove(nxt)
            cur = nxt
        self._visit_queue = ordered
        self._current_box_idx = 0
        self.get_logger().info(
            f"Patrol plan: {len(ordered)} box(es) — "
            + " → ".join(f"({x:.2f},{y:.2f})" for x, y in ordered)
        )

    def _box_label(self) -> str:
        """Human-readable id for the current box: index always, ArUco once known."""
        lbl = f"#{self._current_box_idx}"
        if self._current_box_id is not None:
            lbl += f" (ArUco {self._current_box_id})"
        return lbl

    def _result_key(self):
        """Key used for /inspection_result and _box_results. The ArUco ID once known,
        else a positional fallback for boxes that timed out before any read."""
        return self._current_box_id if self._current_box_id is not None else f"box{self._current_box_idx}"

    def _maybe_start_next_box(self):
        """Pick the next unvisited box or finish the mission."""
        if self._visit_queue:
            nxt = self._visit_queue.pop(0)
            self._current_box_id  = None            # unknown until the bot reads the side ArUco
            self._current_box_idx += 1
            if ARENA_FREE:
                self._current_beacon = nxt          # beacon = box-top marker id
                self.get_logger().info(
                    f"Next target: box {self._box_label()} — homing on beacon {nxt}"
                )
            else:
                self._current_box_pos = nxt
                self.get_logger().info(
                    f"Next target: box {self._box_label()} at "
                    f"({nxt[0]:.2f}, {nxt[1]:.2f})"
                )
            self._transition(FSM.NAVIGATING)
            return

        if self._current_repair_batch:
            self._is_repair_phase = True
            entry = self._current_repair_batch.pop(0)
            if ARENA_FREE:
                self._current_beacon, self._current_box_id = entry
            else:
                self._current_box_pos, self._current_box_id = entry
            self.get_logger().info(f"Next target (REPAIR): box ArUco ID {self._current_box_id}")
            self._transition(FSM.NAVIGATING)
            return

        if self._repair_queue:
            self._transition(FSM.REVIEWING_QUEUE)
            return

        self._transition(FSM.MISSION_COMPLETE)

    # ------------------------------------------------------------------ navigation
    def _navigate_to_current_box(self):
        """Direct camera-guided drive to the pre-dock point (south of the box).
        Replaces Nav2 — in a 2×2 m open arena a simple go-to-goal is far more robust
        than a full planner, which flails into recovery spins at this scale."""
        if ARENA_FREE:
            self.get_logger().info(
                f"Homing (pixel-servo) → box {self._box_label()} beacon {self._current_beacon}"
            )
            self._stop_drive_timer()
            self._drive_acted_seq = -1
            self._drive_pulse_ticks = 0
            self._drive_timer = self.create_timer(0.05, self._drive_tick_px)   # 20 Hz
            return

        pos = self._current_box_pos
        if pos is None:
            self.get_logger().error(f"No position for box {self._box_label()}")
            self._transition(FSM.ALIGNING)
            return
        # Pre-dock point: NAV_STANDOFF_M south of the box; the TOF creep covers the rest.
        self._goal_xy = (pos[0], pos[1] - NAV_STANDOFF_M)
        self.get_logger().info(
            f"Driving (camera-guided) → box {self._box_label()} pre-dock at "
            f"({self._goal_xy[0]:.2f}, {self._goal_xy[1]:.2f})"
        )
        self._stop_drive_timer()
        self._drive_timer = self.create_timer(0.05, self._drive_tick)   # 20 Hz

    def _drive_tick(self):
        if self._state != FSM.NAVIGATING:
            self._stop_drive_timer()
            return
        if self._pose is None or self._goal_xy is None:
            return   # wait until the camera has located the bot

        x, y, yaw = self._pose
        gx, gy = self._goal_xy
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        twist = Twist()

        # Arrival: reached the pre-dock point → align.
        if dist <= GOAL_TOL_M:
            self._cmd_vel_pub.publish(twist)   # stop
            self._stop_drive_timer()
            self.get_logger().info(f"Reached pre-dock (dist={dist*100:.0f}cm) → ALIGNING")
            self._transition(FSM.ALIGNING)
            return

        bearing = math.atan2(dy, dx)
        yaw_err = _wrap(bearing - yaw)
        if abs(yaw_err) > math.radians(DRIVE_TURN_FIRST):
            # Still turning to face the goal — the TOF points off-axis here, so it is NOT a
            # valid obstacle reading yet. Just rotate in place; don't let it abort the drive.
            twist.angular.z = max(-0.6, min(0.6, DRIVE_TURN_KP * yaw_err))
            self._cmd_vel_pub.publish(twist)
            return

        # Pointed at the goal and about to translate — NOW the forward TOF is meaningful.
        if self._tof_mm <= NAV_TOF_SAFETY_MM:
            self._cmd_vel_pub.publish(twist)   # stop
            self._stop_drive_timer()
            self.get_logger().info(
                f"Obstacle/box {self._tof_mm:.0f}mm ahead while facing goal → ALIGNING"
            )
            self._transition(FSM.ALIGNING)
            return
        twist.linear.x  = APPROACH_SPEED
        twist.angular.z = max(-0.4, min(0.4, DRIVE_TURN_KP * yaw_err))
        self._cmd_vel_pub.publish(twist)

    def _stop_drive_timer(self):
        if self._drive_timer is not None:
            self._drive_timer.cancel()
            self.destroy_timer(self._drive_timer)
            self._drive_timer = None

    # ------------------------------------------------------------------ pose
    def _on_pose(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            _wrap(yaw),
        )

    # ------------------------------------------------------------------ pixel-space (arena-free)
    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, _wrap(yaw), time.time())

    def _on_bot_px(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        new_px = (msg.pose.pose.position.x, msg.pose.pose.position.y, _wrap(theta))
        self._fuse_on_camera(new_px)   # feed the online fit, then re-anchor dead-reckoning here
        self._bot_px = new_px
        self._px_seq += 1   # mark a fresh camera frame — the fallback pulse gate keys on this

    # ------------------------------------------------------------------ odom fusion
    def _fuse_on_camera(self, new_px):
        """Called on every fresh camera frame. If the bot moved measurably since the last anchor,
        use the paired (Δodom, Δcamera) as an online sample to fit the odom→image transform; then
        re-anchor the 20 Hz dead-reckoning to this camera pose + the current odom pose (hard snap,
        so accumulated odom drift never exceeds one inter-frame interval)."""
        od = self._odom
        if (ODOM_FUSION and od is not None
                and self._anchor_px is not None and self._anchor_odom is not None):
            dox = od[0] - self._anchor_odom[0]
            doy = od[1] - self._anchor_odom[1]
            if math.hypot(dox, doy) >= FUSE_MIN_TRANS_M:
                # Position sample: odom displacement p ↔ camera pixel displacement q. Accumulate the
                # normal-equation sums with a forgetting factor; A = Sqp·inv(Spp) solved in _refit.
                qu = new_px[0] - self._anchor_px[0]
                qv = new_px[1] - self._anchor_px[1]
                f = FUSE_FORGET
                self._fuse_spp = [f * self._fuse_spp[0] + dox * dox,
                                  f * self._fuse_spp[1] + dox * doy,
                                  f * self._fuse_spp[2] + doy * doy]
                self._fuse_sqp = [f * self._fuse_sqp[0] + qu * dox,
                                  f * self._fuse_sqp[1] + qu * doy,
                                  f * self._fuse_sqp[2] + qv * dox,
                                  f * self._fuse_sqp[3] + qv * doy]
                self._fuse_nsamp += 1
                self._refit_odom_to_image()
            dyaw = _wrap(od[2] - self._anchor_odom[2])
            if abs(dyaw) >= FUSE_MIN_ROT_RAD:
                # Heading-sign sample: Δθ_img vs Δyaw_odom. Sign of the running product resolves the
                # (constant) handedness between odom yaw and image heading.
                dth = _wrap(new_px[2] - self._anchor_px[2])
                self._fuse_hdg_corr = FUSE_FORGET * self._fuse_hdg_corr + dth * dyaw

        self._anchor_px = new_px
        self._anchor_odom = od

    def _refit_odom_to_image(self):
        """Solve the 2×2 odom→image map A = Sqp·inv(Spp) from the running normal-equation sums.
        inv(Spp) = (1/det)·[[syy,-sxy],[-sxy,sxx]] for the symmetric Spp."""
        sxx, sxy, syy = self._fuse_spp
        det = sxx * syy - sxy * sxy
        if abs(det) < FUSE_MIN_DET or self._fuse_nsamp < FUSE_MIN_SAMPLES:
            return
        b11, b12, b21, b22 = self._fuse_sqp
        inv = 1.0 / det
        self._fuse_A = (
            (b11 * syy - b12 * sxy) * inv,
            (-b11 * sxy + b12 * sxx) * inv,
            (b21 * syy - b22 * sxy) * inv,
            (-b21 * sxy + b22 * sxx) * inv,
        )

    def _cur_px(self):
        """Fused pixel pose (u, v, θ): the last camera pose dead-reckoned forward on odom. Falls
        back to the raw camera pose whenever fusion isn't live. Returns None only if the camera has
        never seen the bot. This is the pose source all arena-free steering reads."""
        if self._bot_px is None:
            return None
        if not (ODOM_FUSION and self._px_is_live()):
            return self._bot_px
        od = self._odom
        ax, ay, ayaw, _ = self._anchor_odom
        dox = od[0] - ax
        doy = od[1] - ay
        a11, a12, a21, a22 = self._fuse_A
        u = self._anchor_px[0] + a11 * dox + a12 * doy
        v = self._anchor_px[1] + a21 * dox + a22 * doy
        if self._fuse_hdg_corr != 0.0:
            sign = 1.0 if self._fuse_hdg_corr > 0.0 else -1.0
            theta = _wrap(self._anchor_px[2] + sign * _wrap(od[2] - ayaw))
        else:
            theta = self._anchor_px[2]   # heading sign not yet resolved — hold camera heading
        return (u, v, theta)

    def _px_is_live(self):
        """True when the fused pose is trustworthy at 20 Hz: odom fresh, position fit converged,
        anchors set. Gates continuous control (Phase 5) vs. the fallback per-frame pulse loops."""
        od = self._odom
        return (od is not None and self._anchor_odom is not None and self._anchor_px is not None
                and self._fuse_A is not None and self._fuse_nsamp >= FUSE_MIN_SAMPLES
                and (time.time() - od[3]) <= ODOM_STALE_S)

    def _on_box_px(self, msg: PoseArray):
        """Decode box-top markers: (id, u, v, hdg_rad).
        hdg_rad is the box top ArUco's forward direction in image space — by convention
        this points TOWARD the inspection face.  The bot must approach from that side
        with its nose pointing the opposite way (bot_hdg ≈ box_hdg + π at dock)."""
        result = []
        now = time.time()
        for p in msg.poses:
            bid = int(round(p.position.z))
            u, v = p.position.x, p.position.y
            q = p.orientation
            hdg = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            whdg = _wrap(hdg)
            result.append((bid, u, v, whdg))
            self._beacon_cache[bid] = (u, v, whdg, now)
        self._boxes_px = result
        self._beacon_history.append({bid for bid, *_ in result})

    def _beacon_px(self, bid):
        """Current pixel centre of a box-top marker, or recent cached location if flickering."""
        for mid, u, v, _hdg in self._boxes_px:
            if mid == bid:
                return (u, v)
        if bid in self._beacon_cache:
            u, v, _, t = self._beacon_cache[bid]
            if time.time() - t < 2.0:
                return (u, v)
        return None

    def _box_desired_hdg(self, bid):
        """Heading the bot's nose should point when docked: 180° opposite the box top ArUco.
        Box top ArUco forward → inspection face.  Bot approaches from that side, nose pointing
        back (into the face), so bot_hdg = box_hdg + π."""
        for mid, u, v, hdg in self._boxes_px:
            if mid == bid:
                return _wrap(hdg + math.pi)
        if bid in self._beacon_cache:
            _, _, hdg, t = self._beacon_cache[bid]
            if time.time() - t < 2.0:
                return _wrap(hdg + math.pi)
        return None

    def _beacon_approach_point(self, bid):
        """Target pixel the bot should navigate to: PX_APPROACH_STANDOFF pixels in front of
        the box's inspection face (in the direction the box top ArUco points).
        The bot arrives there with its nose opposite → face-to-face, 180° apart."""
        for mid, u, v, hdg in self._boxes_px:
            if mid == bid:
                au = u + PX_APPROACH_STANDOFF * math.cos(hdg)
                av = v - PX_APPROACH_STANDOFF * math.sin(hdg)   # image Y increases downward
                return (au, av)
        if bid in self._beacon_cache:
            u, v, hdg, t = self._beacon_cache[bid]
            if time.time() - t < 2.0:
                au = u + PX_APPROACH_STANDOFF * math.cos(hdg)
                av = v - PX_APPROACH_STANDOFF * math.sin(hdg)
                return (au, av)
        return None

    def _lateral_box_err(self):
        """Signed lateral offset (in pixels) of the bot marker from the box centerline.
        Centerline is the line through the box top marker along its forward heading.
        +ve = bot is to the RIGHT of the box forward axis.
        -ve = bot is to the LEFT.
        0.0 = centered."""
        cur = self._cur_px()
        if cur is None or self._current_beacon is None:
            return 0.0
        bp = self._beacon_px(self._current_beacon)
        if bp is None:
            return 0.0
        hdg = None
        for mid, _, _, h in self._boxes_px:
            if mid == self._current_beacon:
                hdg = h
                break
        if hdg is None and self._current_beacon in self._beacon_cache:
            hdg = self._beacon_cache[self._current_beacon][2]
        if hdg is None:
            return 0.0

        du = cur[0] - bp[0]
        dv = cur[1] - bp[1]
        # In image coords (+x right, +y down):
        # Unit vector to the right of box heading (90 deg CW):
        # tx = sin(hdg), ty = cos(hdg)
        return du * math.sin(hdg) + dv * math.cos(hdg)

    def _approach_bearing_err(self):
        """Steering error while navigating: angle from bot to the approach point, minus
        bot heading (rad).  Used in _drive_tick_px.  +ve = need to turn CCW."""
        cur = self._cur_px()
        if cur is None or self._current_beacon is None:
            return None
        ap = self._beacon_approach_point(self._current_beacon)
        if ap is None:
            return None
        u, v, theta = cur
        du, dv = ap[0] - u, ap[1] - v
        bearing = math.atan2(-dv, du)   # -dv: image Y increases downward
        return _wrap(bearing - theta)

    def _heading_alignment_err(self):
        """Final alignment error: how far the bot's heading is from the desired dock heading
        (box_hdg + π).  Used in _scan_view_valid and TOF-creep correction.
        +ve = bot needs to turn CCW to align."""
        cur = self._cur_px()
        if cur is None or self._current_beacon is None:
            return None
        desired = self._box_desired_hdg(self._current_beacon)
        if desired is None:
            return None
        _, _, bot_hdg = cur
        return _wrap(desired - bot_hdg)

    # kept for any legacy callers — now delegates to the approach bearing during NAVIGATING
    def _box_bearing_err(self):
        return self._approach_bearing_err()

    def _drive_tick_px(self):
        """Arena-free homing: steer the bot toward the approach point PX_APPROACH_STANDOFF
        pixels in front of the box's inspection face, then hand off to TOF creep.

        Geometry guarantee: the approach point is offset from the box center in the direction
        the box top ArUco points (= toward the inspection face), so the bot naturally arrives
        from that side.  At dock: bot heading ≈ box heading + 180° (face-to-face, 180° apart).
        """
        if self._state != FSM.NAVIGATING:
            self._stop_drive_timer()
            return
        if self._bot_px is None or self._current_beacon is None:
            return

        twist = Twist()
        ap = self._beacon_approach_point(self._current_beacon)
        if ap is None:
            # Box marker not visible this frame — hold still
            self._cmd_vel_pub.publish(twist)
            return

        u, v, _ = self._cur_px()
        gap = math.hypot(ap[0] - u, ap[1] - v)

        # Primary handoff: bot has reached the approach point in front of the box face.
        # Secondary (safety guard): only if close to the approach point (gap <= 60px) and TOF <= 60mm.
        near = gap <= PX_NEAR_GAP or (gap <= 60.0 and USE_TOF and self._tof_mm <= 60.0)
        if near:
            self._cmd_vel_pub.publish(twist)   # stop motors
            self._stop_drive_timer()
            why = f"gap={gap:.0f}px" if gap <= PX_NEAR_GAP else f"TOF safety {self._tof_mm:.0f}mm"
            self.get_logger().info(f"Reached approach point for box {self._box_label()} ({why}) → ALIGNING")
            self._transition(FSM.ALIGNING)
            return

        # Steering error: angle from bot to approach point, relative to bot's current heading.
        yaw_err = self._approach_bearing_err()
        if yaw_err is None:
            return

        if self._px_is_live():
            # Continuous arc follower on the 20 Hz fused pose: forward speed scaled by cos(bearing
            # error) so the bot smoothly pivots when far off-heading and arcs in once roughly aimed —
            # no stop-turn-go stutter. (On the raw ~1 Hz camera this oscillates, hence the fallback.)
            twist.linear.x  = PX_DRIVE_SPEED * max(0.0, math.cos(yaw_err))
            twist.angular.z = max(-PX_ANG_MAX, min(PX_ANG_MAX, PX_TURN_KP * yaw_err * PX_TURN_SIGN))
            self._cmd_vel_pub.publish(twist)
            return

        # Fallback (odom fusion not live yet): turn in place as ONE pulse per fresh camera frame.
        # The overhead stream is ~1 Hz while this timer is 20 Hz; turning every tick fires ~6
        # open-loop pulses on the same stale pose, so the bot whips past the target and flails.
        if abs(yaw_err) > math.radians(PX_TURN_FIRST_DEG):
            if self._drive_pulse_ticks > 0:
                self._drive_pulse_ticks -= 1
                twist.angular.z = self._drive_pulse_cmd
                self._cmd_vel_pub.publish(twist)
                return
            if self._px_seq == self._drive_acted_seq:
                self._cmd_vel_pub.publish(twist)   # no fresh frame yet — stay still
                return
            self._drive_acted_seq = self._px_seq
            direction = 1.0 if (yaw_err * PX_TURN_SIGN) > 0 else -1.0
            self._drive_pulse_cmd = 0.5 * direction
            self._drive_pulse_ticks = max(1, min(6, int(round(math.degrees(abs(yaw_err)) / 10.0))))
            self._drive_pulse_ticks -= 1
            twist.angular.z = self._drive_pulse_cmd
            self._cmd_vel_pub.publish(twist)
            return

        # Aimed closely enough: drive forward with gentle proportional steer.
        twist.linear.x  = PX_DRIVE_SPEED
        twist.angular.z = max(-0.4, min(0.4, PX_TURN_KP * yaw_err * PX_TURN_SIGN))
        self._cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------ alignment
    def _start_align(self):
        """Center the wheel-axle (rotation centre) on the box centerline, then face the box.
        In arena-free mode: rotate in place until bot heading is 180° opposite box top ArUco."""
        self._stop_align_timer()   # guard against a stale timer on retry
        self._align_ok_count = 0
        self._align_acted_seq = -1
        self._align_pulse_ticks = 0
        self._align_pulse_count = 0
        self._align_settle_ticks = 0
        self._align_stage = "heading"
        self._align_round = 0
        if ARENA_FREE and MOVE_PRIMITIVES:
            self._start_align_prim()
            return
        if ARENA_FREE:
            desired = self._box_desired_hdg(self._current_beacon)
            des_deg = math.degrees(desired) if desired is not None else 0.0
            self.get_logger().info(
                f"Aligning (arena-free): rotating in place to face box {self._box_label()} "
                f"(desired hdg={des_deg:.1f}°, target ±{PERP_TOL_DEG:.1f}°)"
            )
            self._align_timer = self.create_timer(0.05, self._align_tick_px)   # 20 Hz
            return

        pos = self._current_box_pos
        need_center = (self._pose is not None and pos is not None
                       and abs(pos[0] - _axle_x(self._pose[0], self._pose[2])) > LATERAL_TOL_M)
        self._align_phase = "center_turn" if need_center else "perp_turn"
        self.get_logger().info(
            f"Aligning (phase={self._align_phase}, target ±{PERP_TOL_DEG:.1f}°)"
        )
        self._align_timer = self.create_timer(0.05, self._align_tick)   # 20 Hz

    def _align_tick_px(self):
        """Arena-free alignment, staged so the bot ends up BOTH square-on and centred on the box
        marker (the line joining the two marker centres perpendicular to the box face):

            heading   → rotate to box_hdg+π within ±PERP_TOL_DEG
            lat_check → measure offset from the box's forward axis; centred → done
            lat_turn  → turn to face the sideways direction that kills that offset
            lat_drive → nudge forward to cancel the offset, then re-do heading

        It loops heading↔lateral until both are satisfied (or ALIGN_MAX_ROUNDS, then best-effort).

        Every rotation is PULSE-AND-SETTLE, never continuous: udp_bridge floors any nonzero wheel
        command up to MIN_MOVE_PWM, so a 'gentle' continuous turn is really a full-speed spin. The
        only way to place a heading finely is ONE short pulse (duration, not speed, sets the angle),
        motors OFF until a fresh camera pose shows where it landed, then re-measure."""
        if self._state != FSM.ALIGNING:
            self._stop_align_timer()
            return
        if self._bot_px is None or self._current_beacon is None:
            return

        stage = self._align_stage
        if stage == "heading":
            self._align_stage_heading()
        elif stage == "lat_check":
            self._align_stage_lat_check()
        elif stage == "lat_turn":
            self._align_stage_lat_turn()
        elif stage == "lat_drive":
            self._align_stage_lat_drive()

    def _set_align_stage(self, stage):
        """Switch staged-align phase and reset the per-stage pulse accounting."""
        self._align_stage = stage
        self._align_ok_count = 0
        self._align_acted_seq = -1
        self._align_pulse_ticks = 0
        self._align_pulse_count = 0
        self._align_settle_ticks = 0

    def _pulse_rotate(self, err, tol_rad):
        """One error-sized turn pulse per fresh camera frame, driving err→0. The firmware floors the
        command to a spin, so DURATION sets the angle; between pulses motors are off until a new pose
        arrives (so err reflects the last pulse, not a stale heading). Returns 'done' (within tol on
        two consecutive fresh frames), 'giveup' (exceeded ALIGN_MAX_PULSES) or 'work'."""
        twist = Twist()
        if self._align_pulse_ticks > 0:
            self._align_pulse_ticks -= 1
            twist.angular.z = self._align_pulse_cmd
            self._cmd_vel_pub.publish(twist)
            return "work"
        # Settle: hold still long enough for the spin + link latency to clear, so the error we read
        # next reflects the pulse we just made — not the bot still coasting from it.
        if self._align_settle_ticks > 0:
            self._align_settle_ticks -= 1
            self._cmd_vel_pub.publish(twist)
            return "work"
        if self._px_seq == self._align_acted_seq:
            self._cmd_vel_pub.publish(twist)
            return "work"
        if abs(err) <= tol_rad:
            self._cmd_vel_pub.publish(twist)
            self._align_acted_seq = self._px_seq
            self._align_ok_count += 1
            return "done" if self._align_ok_count >= 2 else "work"
        self._align_ok_count = 0
        self._align_acted_seq = self._px_seq
        self._align_pulse_count += 1
        if self._align_pulse_count > ALIGN_MAX_PULSES:
            return "giveup"
        direction = 1.0 if (err * PX_TURN_SIGN) > 0 else -1.0
        self._align_pulse_cmd = 0.4 * direction
        self._align_pulse_ticks = max(1, min(6, int(round(math.degrees(abs(err)) / 10.0))))
        self._align_settle_ticks = ALIGN_SETTLE_TICKS
        self._align_pulse_ticks -= 1
        twist.angular.z = self._align_pulse_cmd
        self._cmd_vel_pub.publish(twist)
        return "work"

    def _align_stage_heading(self):
        err = self._heading_alignment_err()
        if err is None:
            self._cmd_vel_pub.publish(Twist())
            return
        r = self._pulse_rotate(err, math.radians(PERP_TOL_DEG))
        if r == "giveup":
            self.get_logger().warn(
                f"Heading for box {self._box_label()} didn't settle within ±{PERP_TOL_DEG:.0f}° "
                f"(err={math.degrees(err):.1f}°) — checking centring anyway"
            )
            self._set_align_stage("lat_check")
        elif r == "done":
            self._set_align_stage("lat_check")

    def _align_stage_lat_check(self):
        self._cmd_vel_pub.publish(Twist())
        lat = self._lateral_box_err()
        if abs(lat) <= LAT_TOL_PX:
            self._finish_align(self._heading_alignment_err() or 0.0)
            return
        if self._align_round >= ALIGN_MAX_ROUNDS:
            self.get_logger().warn(
                f"Box {self._box_label()} not centred after {ALIGN_MAX_ROUNDS} rounds "
                f"(lateral {lat:.0f}px) — proceeding best-effort"
            )
            self._finish_align(self._heading_alignment_err() or 0.0)
            return
        self._align_round += 1
        self._lat_sign = 1.0 if lat > 0 else -1.0
        self.get_logger().info(
            f"Centring on box {self._box_label()}: lateral offset {lat:.0f}px "
            f"(round {self._align_round}/{ALIGN_MAX_ROUNDS})"
        )
        self._set_align_stage("lat_turn")

    def _lat_drive_hdg(self):
        """Heading the bot should face to drive forward and cancel the current lateral offset:
        straight along the box's lateral (perpendicular-to-forward) axis, toward the centerline."""
        desired = self._box_desired_hdg(self._current_beacon)
        if desired is None:
            return None
        h = _wrap(desired - math.pi)   # box forward heading (toward the inspection face)
        s = self._lat_sign
        return _wrap(math.atan2(s * math.cos(h), -s * math.sin(h)))

    def _align_stage_lat_turn(self):
        tgt = self._lat_drive_hdg()
        cur = self._cur_px()
        if tgt is None or cur is None:
            self._cmd_vel_pub.publish(Twist())
            return
        err = _wrap(tgt - cur[2])
        r = self._pulse_rotate(err, math.radians(LAT_TURN_TOL_DEG))
        if r in ("done", "giveup"):
            self._set_align_stage("lat_drive")

    def _align_stage_lat_drive(self):
        # Nudge straight forward (we are facing the sideways direction) to walk the lateral offset to
        # zero, one fresh-frame-gated pulse at a time. Stop on centred, on overshoot (sign flip), or
        # the pulse cap — then go back to heading, which re-checks centring after.
        twist = Twist()
        lat = self._lateral_box_err()
        if abs(lat) <= LAT_TOL_PX or (lat * self._lat_sign) < 0:
            self._cmd_vel_pub.publish(twist)
            self._set_align_stage("heading")
            return
        if self._align_pulse_ticks > 0:
            self._align_pulse_ticks -= 1
            twist.linear.x = PX_DRIVE_SPEED
            self._cmd_vel_pub.publish(twist)
            return
        if self._align_settle_ticks > 0:   # let the nudge + link latency clear before re-measuring
            self._align_settle_ticks -= 1
            self._cmd_vel_pub.publish(twist)
            return
        if self._px_seq == self._align_acted_seq:
            self._cmd_vel_pub.publish(twist)
            return
        self._align_acted_seq = self._px_seq
        self._align_pulse_count += 1
        if self._align_pulse_count > ALIGN_MAX_PULSES:
            self._cmd_vel_pub.publish(twist)
            self._set_align_stage("heading")
            return
        self._align_pulse_ticks = max(1, min(4, int(abs(lat) / 10.0)))
        self._align_settle_ticks = ALIGN_SETTLE_TICKS
        self._align_pulse_ticks -= 1
        twist.linear.x = PX_DRIVE_SPEED
        self._cmd_vel_pub.publish(twist)

    def _finish_align(self, err):
        self._stop_align_timer()
        cur = self._cur_px()
        bot_hdg = cur[2] if cur is not None else 0.0
        desired = self._box_desired_hdg(self._current_beacon)
        des_deg = math.degrees(desired) if desired is not None else 0.0
        # Inspect pass reads at the wider scan standoff (whole box face in the S3 FOV); only the
        # repair pass creeps all the way in to the dock distance. Point-blank reading edge-clips the
        # marker, which is why a near-perfect dock could still miss it.
        nxt = FSM.APPROACH_DOCK if self._is_repair_phase else FSM.APPROACH_SCAN
        self.get_logger().info(
            f"Aligned face-to-face with box {self._box_label()}! "
            f"bot={math.degrees(bot_hdg):.1f}°, target={des_deg:.1f}° "
            f"(err={math.degrees(err):.1f}°) → {nxt}"
        )
        self._transition(nxt)

    def _align_tick(self):
        if self._state != FSM.ALIGNING:
            self._stop_align_timer()
            return
        if self._pose is None:
            return
        pos = self._current_box_pos
        if pos is None:
            return

        x, _, yaw = self._pose
        e_axle = pos[0] - _axle_x(x, yaw)   # +ve → box centre is east (+x) of the axle
        twist = Twist()

        if self._align_phase == "center_turn":
            if abs(e_axle) <= LATERAL_TOL_M:
                self._align_phase = "perp_turn"
                return
            # Face along the centerline correction direction (east or west)
            target = 0.0 if e_axle > 0 else math.pi
            err = _wrap(target - yaw)
            if abs(err) <= math.radians(2.0):
                self._align_phase = "center_drive"
            else:
                twist.angular.z = max(-0.8, min(0.8, ALIGN_YAW_KP * err))
                self._cmd_vel_pub.publish(twist)
            return

        if self._align_phase == "center_drive":
            # Driving forward while facing east/west shrinks the axle's lateral offset
            if abs(e_axle) <= 0.01:
                self._cmd_vel_pub.publish(twist)   # stop
                self._align_phase = "perp_turn"
            else:
                twist.linear.x = APPROACH_SPEED
                self._cmd_vel_pub.publish(twist)
            return

        # perp_turn — rotates about the axle, so the centered axle_x is preserved
        err = _wrap(TARGET_YAW - yaw)
        if abs(err) <= math.radians(PERP_TOL_DEG):
            self._align_ok_count += 1
        else:
            self._align_ok_count = 0
        if self._align_ok_count >= 5:          # hold in-tolerance before committing
            self._cmd_vel_pub.publish(twist)   # stop
            self._stop_align_timer()
            # Repair pass skips inspection and docks straight in; scan pass reads first
            if self._is_repair_phase:
                self._transition(FSM.APPROACH_DOCK)
            else:
                self._transition(FSM.APPROACH_SCAN)
            return
        twist.angular.z = max(-0.6, min(0.6, ALIGN_YAW_KP * err))
        self._cmd_vel_pub.publish(twist)

    def _stop_align_timer(self):
        if self._align_timer is not None:
            self._align_timer.cancel()
            self.destroy_timer(self._align_timer)
            self._align_timer = None

    # ------------------------------------------------------------------ approach
    def _on_tof(self, msg: Range):
        self._tof_mm = msg.range * 1000.0
        if MOVE_PRIMITIVES and ARENA_FREE:
            return   # the primitive approach machine reads _tof_mm itself; it owns the motors
        if not USE_TOF:
            return   # TOF unreliable — approach/dock are driven by the pixel gap instead
        if self._state not in (FSM.APPROACH_SCAN, FSM.APPROACH_DOCK):
            return

        target_mm = SCAN_STOP_MM if self._state == FSM.APPROACH_SCAN else DOCK_STOP_MM
        twist = Twist()
        tof_sane = 30.0 <= self._tof_mm < 2000.0

        # Settle-back: after the dock stop, reverse-creep until the TOF shows a reliable gap
        # >= DOCK_STOP_MM. Backing away from the box walks the reading up through the VL53L0X's
        # trustworthy band, so every run ends at the same readable ~10 cm stand-off — regardless
        # of whether we stopped on-target or rammed in past the TOF dead zone.
        if self._dock_settling:
            if tof_sane and self._tof_mm >= DOCK_STOP_MM:
                self._cmd_vel_pub.publish(twist)   # stop
                self._send_cmd("STOP")
                self._dock_settling = False
                self.get_logger().info(
                    f"Settled at {self._tof_mm:.0f} mm — readable gap for box {self._box_label()} → "
                    f"{'repair' if self._is_repair_phase else 'inspect'}"
                )
                self._finish_dock()
            else:
                twist.linear.x = -DOCK_CREEP_SPEED   # too close / saturated: back away slowly
                self._cmd_vel_pub.publish(twist)
            return

        # Overhead-camera gap, kept ONLY as a loose emergency backstop (not the normal stop).
        # ~118 px ≈ touching, ~6.5 px/cm in the overhead view.
        dist_px = None
        if ARENA_FREE and self._bot_px is not None and self._current_beacon is not None:
            bp = self._beacon_px(self._current_beacon)
            if bp is not None:
                dist_px = math.hypot(bp[0] - self._bot_px[0], bp[1] - self._bot_px[1])

        # TOF is the sole primary dock-stop. We stop at ~10 cm (DOCK_STOP_MM), which is inside the
        # VL53L0X's reliable range, so the noisy camera-pixel scale is NOT used as the normal stop.
        # Trigger STOP a few mm BEFORE the target so the bot's coast lands it at ~target, not past it.
        tof_reached = tof_sane and self._tof_mm <= (target_mm + DOCK_ANTICIPATE_MM)
        # Camera backstop fires ONLY when the TOF is in its saturated <5 cm dead zone (or garbage):
        # that means we already overshot, so stop at the first sign of it.
        overshoot = not tof_sane and dist_px is not None and dist_px <= (118.0 + 60.0 * 0.65)

        dock_stop = tof_reached or overshoot
        if dock_stop:
            self._cmd_vel_pub.publish(twist)   # stop
            self._send_cmd("STOP")
            reason = "TOF" if tof_reached else "camera overshoot backstop"
            if self._state == FSM.APPROACH_SCAN:
                self._transition(FSM.INSPECTING)
            else:
                dist_str = f", dist={dist_px:.0f}px" if dist_px is not None else ""
                self.get_logger().info(
                    f"Dock stop at box {self._box_label()} via {reason} "
                    f"(TOF {self._tof_mm:.0f} mm{dist_str}) — settling to {DOCK_STOP_MM:.0f} mm"
                )
                if DOCK_SETTLE:
                    self._dock_settling = True   # back off to a guaranteed readable gap, then finish
                else:
                    self._finish_dock()
        else:
            # Two-speed creep: full approach speed until the near band, then crawl so that crossing
            # the stop threshold doesn't carry the bot several cm into the box on momentum alone.
            in_near_band = tof_sane and self._tof_mm <= (target_mm + DOCK_NEAR_BAND_MM)
            twist.linear.x = DOCK_CREEP_SPEED if in_near_band else APPROACH_SPEED
            # Hold heading and center on the box while creeping so the camera lands dead-center on the ArUco.
            if ARENA_FREE:
                heading_err = self._heading_alignment_err()
                if heading_err is not None:
                    # Active lateral centering:
                    # lat_err > 0 means bot is to the RIGHT of the box centerline (from box's view).
                    # Steer slightly left (negative offset to angle) to glide into the center.
                    lat_err = self._lateral_box_err()
                    # 1 px error ≈ 0.012 rad (~0.7 deg) correction, capped at ±15 deg
                    lat_corr = max(-math.radians(15.0), min(math.radians(15.0), -0.012 * lat_err))
                    steer_err = _wrap(heading_err + lat_corr)
                    twist.angular.z = max(-0.35, min(0.35, PX_TURN_KP * steer_err * PX_TURN_SIGN))
            elif self._pose is not None:
                # (metric path) lateral centering was done in ALIGNING via the axle.
                yaw_err = _wrap(TARGET_YAW - self._pose[2])
                twist.angular.z = max(-0.3, min(0.3, APPROACH_YAW_KP * yaw_err))
            self._cmd_vel_pub.publish(twist)

    def _finish_dock(self):
        if self._is_repair_phase:
            self._transition(FSM.REPAIRING)
        else:
            self._transition(FSM.INSPECTING)

    # ------------------------------------------------------------------ inspection
    def _start_inspect_backoff(self):
        """Distance back-off read: reverse-creep straight out in ~1 cm steps, pausing to look at each,
        until the S3 confirms the marker. The bot is already square-on and centred from ALIGNING +
        the TOF creep, so backing straight keeps the marker centred in the S3 FOV — we just walk the
        distance into the ~18-22 cm band where the whole marker reads cleanly."""
        self._stop_inspect_backoff()
        start_mm = self._tof_mm if (30.0 <= self._tof_mm < 2000.0) else SCAN_STOP_MM
        self._backoff_target = start_mm + INSPECT_BACKOFF_STEP_MM
        self._backoff_settle = INSPECT_BACKOFF_SETTLE_TICKS   # look where we are before the first step
        self._backoff_hold   = INSPECT_BACKOFF_HOLD_TICKS
        self._backoff_stream_dead = False
        self._backoff_stale_logged = False
        self.get_logger().info(
            f"Inspect back-off for box {self._box_label()}: reading from {start_mm:.0f} mm out to "
            f"{INSPECT_BACKOFF_MAX_MM:.0f} mm in {INSPECT_BACKOFF_STEP_MM:.0f} mm steps"
        )
        self._backoff_timer = self.create_timer(0.05, self._inspect_backoff_tick)   # 20 Hz

    def _inspect_backoff_tick(self):
        # SETTLE (motors off, let the S3 decode) then STEP (reverse until TOF >= target). A confirmed
        # read is handled entirely by _on_aruco_detected, which halts this timer the instant it fires.
        if self._state != FSM.INSPECTING:
            self._cmd_vel_pub.publish(Twist())
            self._stop_inspect_backoff()
            return

        twist = Twist()

        # Stream-liveness gate: never step the search on a dead S3 feed, or the band "completes" while
        # looking at nothing and a FAULT box gets silently cleared. Hold still until frames resume.
        if (time.monotonic() - self._last_cam_alive) > INSPECT_STREAM_STALE_S:
            self._backoff_stream_dead = True
            if not self._backoff_stale_logged:
                self.get_logger().warn(
                    f"Inspection paused for box {self._box_label()} — S3 stream stale "
                    f"(no frames >{INSPECT_STREAM_STALE_S:.0f}s); holding until it recovers"
                )
                self._backoff_stale_logged = True
            self._cmd_vel_pub.publish(twist)
            return
        self._backoff_stale_logged = False

        # Settle window: motors off so the bot is physically still (sharp S3 frame) and bot_cam's
        # confirmation window can commit before we move again.
        if self._backoff_settle > 0:
            self._backoff_settle -= 1
            self._cmd_vel_pub.publish(twist)
            return

        # Reached the readable-band limit with nothing confirmed. Don't quit instantly — a laggy S3
        # often confirms a few hundred ms after the band is walked (a real FAULT read was thrown away
        # by ~250 ms once). Hold at the far standoff and keep looking (stream stays live via the gate
        # above) for the grace window, then declare the miss.
        if self._backoff_target > INSPECT_BACKOFF_MAX_MM:
            if self._backoff_hold > 0:
                self._backoff_hold -= 1
                self._cmd_vel_pub.publish(twist)
                return
            self._inspect_miss("backed off through readable band + grace, no marker")
            return

        tof_sane = 30.0 <= self._tof_mm < 2000.0
        if tof_sane and self._tof_mm >= self._backoff_target:
            # Stepped far enough: stop, settle, then look here before stepping again.
            self._backoff_target += INSPECT_BACKOFF_STEP_MM
            self._backoff_settle = INSPECT_BACKOFF_SETTLE_TICKS
            self._cmd_vel_pub.publish(twist)
            return

        # Reverse DEAD straight — equal wheel speeds, zero angular. The bot entered INSPECTING already
        # square-on, and over the ~16 cm back-off any closed-loop heading term (at creep speed, a few
        # degrees of error becomes a ~6 cm-radius pivot) swings the nose and traces an arc off the box
        # face. Exactly like the dock-settle reverse in _on_tof: straight out, no steering.
        twist.linear.x = -DOCK_CREEP_SPEED
        self._cmd_vel_pub.publish(twist)

    def _stop_inspect_backoff(self):
        if self._backoff_timer is not None:
            self._backoff_timer.cancel()
            self.destroy_timer(self._backoff_timer)
            self._backoff_timer = None

    def _start_inspect_sweep(self):
        self._stop_inspect_sweep()
        # Deterministic full-arc search — no guessing which way the marker sits. The bot pans about
        # its axle through scripted waypoints (+AMPL → −AMPL → centre) so the S3 sweeps the ENTIRE
        # plausible face either side of the dock heading. It stops the instant bot_cam confirms a
        # marker; only if a whole cycle covers the arc with the stream alive and nothing is seen is
        # it a real miss. Pulse-and-pause (not a continuous spin): MIN_MOVE_PWM floors any command
        # into a fast spin, so we fire one short pulse then hold motors OFF so the bot is physically
        # still (sharp S3 frame) and the overhead heading can refresh before the next pulse.
        cur = self._cur_px()
        self._sweep_center_hdg = cur[2] if cur is not None else None
        self._sweep_idx      = 0            # waypoint index into the scripted arc
        self._sweep_cycles   = 0
        self._sweep_leg_pulses = 0
        self._sweep_wp_dir   = None         # latched travel direction for the active leg
        self._sweep_cooldown = 0
        self._sweep_stream_dead = False
        self._sweep_stale_logged = False
        self._sweep_timer = self.create_timer(0.05, self._inspect_sweep_tick)   # 20 Hz

    def _inspect_sweep_tick(self):
        # Walk the scripted arc waypoints, pausing motors-off between pulses. _on_aruco_detected
        # halts this the instant a marker is confirmed; it self-stops once the state leaves INSPECTING.
        if self._state != FSM.INSPECTING:
            self._cmd_vel_pub.publish(Twist())
            self._stop_inspect_sweep()
            return

        twist = Twist()

        # Stream-liveness gate: never advance the search on a dead S3 feed. Otherwise the arc
        # "completes" while looking at nothing and a FAULT box gets silently cleared. Hold still
        # and wait for frames to resume — the whole point of the heartbeat.
        if (time.monotonic() - self._last_cam_alive) > INSPECT_STREAM_STALE_S:
            self._sweep_stream_dead = True
            if not self._sweep_stale_logged:
                self.get_logger().warn(
                    f"Inspection paused for box {self._box_label()} — S3 stream stale "
                    f"(no frames >{INSPECT_STREAM_STALE_S:.0f}s); holding until it recovers"
                )
                self._sweep_stale_logged = True
            self._cmd_vel_pub.publish(twist)
            return
        self._sweep_stale_logged = False

        # Pause window: motors OFF so the bot settles (sharp S3 frame) and a fresh pose arrives.
        if self._sweep_cooldown > 0:
            self._sweep_cooldown -= 1
            self._cmd_vel_pub.publish(twist)
            return

        targets = [math.radians(INSPECT_SWEEP_AMPL), -math.radians(INSPECT_SWEEP_AMPL), 0.0]
        target  = targets[self._sweep_idx]

        cur = self._cur_px()
        offset = (_wrap(cur[2] - self._sweep_center_hdg)
                  if (cur is not None and self._sweep_center_hdg is not None) else None)

        # Latch this leg's travel direction (toward the target) on entry.
        if self._sweep_wp_dir is None:
            if offset is not None:
                self._sweep_wp_dir = 1.0 if (target - offset) >= 0 else -1.0
            else:
                self._sweep_wp_dir = 1.0 if target >= 0 else -1.0

        # Reached the waypoint? Prefer the MEASURED heading (crossed/landed on the target in the
        # travel direction); fall back to a per-leg pulse cap when the overhead pose is stale, so
        # the leg still terminates and the search can't stall open-loop.
        reached = (offset is not None
                   and (target - offset) * self._sweep_wp_dir <= math.radians(INSPECT_SWEEP_WP_TOL_DEG))
        if self._sweep_leg_pulses >= INSPECT_SWEEP_MAX_PULSES:
            reached = True

        if reached:
            self._sweep_idx += 1
            self._sweep_leg_pulses = 0
            self._sweep_wp_dir = None
            if self._sweep_idx >= len(targets):
                self._sweep_idx = 0
                self._sweep_cycles += 1
                if self._sweep_cycles >= INSPECT_SWEEP_CYCLES:
                    self._inspect_miss("swept full arc, no readable marker")
                    return
            self._sweep_cooldown = INSPECT_SWEEP_PAUSE_TICKS   # settle before the next leg
            self._cmd_vel_pub.publish(twist)
            return

        twist.angular.z = INSPECT_SWEEP_SPEED * self._sweep_wp_dir * PX_TURN_SIGN
        self._cmd_vel_pub.publish(twist)
        self._sweep_leg_pulses += 1
        self._sweep_cooldown = INSPECT_SWEEP_PAUSE_TICKS

    def _stop_inspect_sweep(self):
        if self._sweep_timer is not None:
            self._sweep_timer.cancel()
            self.destroy_timer(self._sweep_timer)
            self._sweep_timer = None

    def _on_bot_cam_status(self, msg: String):
        # Any heartbeat means the S3 is actually delivering frames right now; the inspection sweep
        # only advances while this stays fresh (see the stale gate in _inspect_sweep_tick).
        self._last_cam_alive = time.monotonic()

    def _start_inspection(self):
        self._send_cmd("SCAN")
        self._inspect_retries = 0
        if INSPECT_BACKOFF:
            search = (f"distance back-off (→{INSPECT_BACKOFF_MAX_MM:.0f} mm "
                      f"in {INSPECT_BACKOFF_STEP_MM:.0f} mm steps)")
        else:
            search = f"full-arc sweep ({INSPECT_SWEEP_CYCLES}× ±{INSPECT_SWEEP_AMPL:.0f}°)"
        self.get_logger().info(
            f"SCAN sent for box {self._box_label()} — {search}, gated on a live S3 stream"
        )
        # Absolute wall-clock backstop: the search drives completion itself, but if it somehow stalls
        # (e.g. the stream flaps in and out indefinitely) this guarantees the FSM never hangs.
        self._inspect_timer = self.create_timer(INSPECT_SAFETY_S, self._inspect_safety)

    def _cancel_inspect_timer(self):
        if self._inspect_timer is not None:
            self._inspect_timer.cancel()
            self.destroy_timer(self._inspect_timer)
            self._inspect_timer = None

    def _cancel_precapture_timer(self):
        if self._precapture_timer is not None:
            self._precapture_timer.cancel()
            self.destroy_timer(self._precapture_timer)
            self._precapture_timer = None

    def _take_precapture(self):
        """Return a fresh, valid pre-captured box ArUco (and clear the cache), or None."""
        cid = self._precaptured_aruco
        self._precaptured_aruco = None
        if not INSPECT_PRECAPTURE or cid is None:
            return None
        if (time.monotonic() - self._precapture_ts) > INSPECT_PRECAPTURE_TTL_S:
            return None
        if cid not in BOX_ARUCO_IDS:
            return None
        return cid

    def _fire_precapture(self, cid: int):
        self._cancel_precapture_timer()
        if self._state != FSM.INSPECTING:   # only ever resolve a verdict while actually inspecting
            return
        self._cancel_inspect_timer()
        self._cmd_vel_pub.publish(Twist())   # bot is parked at the stop — ensure it holds still
        self._current_box_id = cid
        self._resolve_inspection(cid)

    def _inspect_safety(self):
        self._cancel_inspect_timer()
        if self._state != FSM.INSPECTING:
            return
        self._inspect_miss("safety timeout — inspection did not complete")

    def _on_aruco_detected(self, msg: String):
        try:
            detected_id = int(msg.data)
        except ValueError:
            return
        if detected_id not in BOX_ARUCO_IDS:
            return

        # The bot usually reads the marker while it's squaring up (ALIGNING) or creeping in
        # (APPROACH_SCAN) — and bot_cam only republishes on *change*, so a read latched during
        # ALIGNING is never re-emitted at INSPECTING. Cache it now (last-write-wins: the final,
        # most square-on read before the stop is the trustworthy one) and replay it at INSPECTING.
        if INSPECT_PRECAPTURE and self._state in (FSM.ALIGNING, FSM.APPROACH_SCAN):
            self._precaptured_aruco = detected_id
            self._precapture_ts = time.monotonic()
            self.get_logger().info(
                f"Pre-captured ArUco {detected_id} (during {self._state}) — will replay at INSPECTING"
            )
            return

        if self._state != FSM.INSPECTING:
            return
        self._cancel_inspect_timer()
        self._cancel_precapture_timer()
        self._stop_inspect_sweep()
        self._stop_inspect_backoff()
        self._abort_primitive()
        self._stop_sampling()
        self._cmd_vel_pub.publish(Twist())   # halt the pan the instant the marker is read
        # Identity is discovered here — the overhead camera only knew this box as a
        # position; the bot's S3 camera reads the actual ArUco ID at the box face.
        self._current_box_id = detected_id
        self._resolve_inspection(detected_id)

    def _inspect_miss(self, reason: str):
        self._cancel_inspect_timer()
        self._cancel_precapture_timer()
        if self._state != FSM.INSPECTING:
            return
        self._stop_inspect_sweep()
        self._stop_inspect_backoff()
        self._abort_primitive()
        self._stop_sampling()
        self._cmd_vel_pub.publish(Twist())
        self._inspect_retries = 0
        key = self._result_key()
        # A miss is NOT an OK. Either the stream was dead or we swept the whole face and found no
        # marker — both are UNREADABLE and must be surfaced, never silently passed, else a genuine
        # FAULT box clears inspection just because the camera couldn't see it.
        stream_note = (" (S3 stream delivered no frames)"
                       if (self._sweep_stream_dead or self._backoff_stream_dead) else "")
        self.get_logger().warn(
            f"Inspection INCONCLUSIVE for box {self._box_label()}: {reason}{stream_note} → UNKNOWN"
        )
        self._box_results[key] = "UNKNOWN"
        self._result_pub.publish(String(data=f"UNKNOWN:{key}"))
        self._transition(FSM.IDLE)

    def _scan_view_valid(self) -> bool:
        """True only if the bot is square-on (±PERP_TOL_DEG), centred on the box
        (±LATERAL_TOL_M) and within scan range — i.e. the whole QR is readable."""
        if ARENA_FREE:
            dist_px = None
            if self._bot_px is not None and self._current_beacon is not None:
                bp = self._beacon_px(self._current_beacon)
                if bp is not None:
                    dist_px = math.hypot(bp[0] - self._bot_px[0], bp[1] - self._bot_px[1])
            target_optical_px = 118.0 + (DOCK_STOP_MM * 0.65)
            dist_ok = (not USE_TOF) or (self._tof_mm <= (SCAN_STOP_MM + 50)) or (dist_px is not None and dist_px <= (target_optical_px + 30.0))

            # The active search (back-off or sweep) reads the marker while the bot is deliberately off
            # the nominal dock pose — panned, or backed out past the scan stop — so a heading/distance
            # gate here would reject exactly the read the search is built to catch. The decode is
            # already trustworthy: the bot only reached INSPECTING by a TOF-gated square-on dock, and
            # bot_cam confirms the NEAREST marker's ID across several frames — a 4×4 ArUco decodes to
            # the right ID or not at all. So a confirmed search decode is accepted outright.
            if INSPECT_BACKOFF or INSPECT_SWEEP:
                mode = "back-off" if INSPECT_BACKOFF else "sweep"
                self.get_logger().info(f"view check box {self._current_box_id}: {mode} read → OK")
                return True

            # Non-sweep pixel path: require the face-on dock heading within PX_AIM_TOL_DEG.
            err = self._heading_alignment_err()
            if err is None:
                return False
            ok = (abs(err) <= math.radians(PX_AIM_TOL_DEG) and dist_ok)
            dist_desc = f"{self._tof_mm:.0f}mm" if (dist_px is None or self._tof_mm < 8000) else f"{dist_px:.0f}px"
            self.get_logger().info(
                f"view check box {self._current_box_id}: "
                f"hdg_err={math.degrees(err):.1f}° dist={dist_desc} "
                f"→ {'OK' if ok else 'FAIL'}"
            )
            return ok

        if self._pose is None:
            return False
        pos = self._current_box_pos
        if pos is None:
            return False
        x, _, yaw = self._pose
        perp_err = abs(_wrap(yaw - TARGET_YAW))
        lat_err  = abs(_camera_x(x, yaw) - pos[0])   # camera's view, not base_footprint
        dist_ok  = self._tof_mm <= (SCAN_STOP_MM + 30)
        ok = (perp_err <= math.radians(PERP_TOL_DEG)
              and lat_err <= LATERAL_TOL_M
              and dist_ok)
        self.get_logger().info(
            f"view check box {self._current_box_id}: "
            f"perp={math.degrees(perp_err):.2f}° lat={lat_err*100:.1f}cm "
            f"dist={self._tof_mm:.0f}mm → {'OK' if ok else 'FAIL'}"
        )
        return ok

    def _resolve_inspection(self, detected_id: int):
        if self._state != FSM.INSPECTING:
            return

        # Gate on geometry: a partial/angled view must not resolve to a verdict
        if not self._scan_view_valid():
            self._inspect_retries += 1
            if self._inspect_retries <= 2:
                self.get_logger().warn(
                    f"ArUco ID {detected_id} seen but view geometry invalid for box "
                    f"{self._box_label()} (retry {self._inspect_retries}/2) — re-approaching"
                )
                self._transition(FSM.ALIGNING)
                return
            self._inspect_retries = 0
            self._inspect_miss("view geometry never valid")
            return

        self._inspect_retries = 0
        verdict = fault_config.get(detected_id, "OK")
        self.get_logger().info(
            f"Inspection result for box {self._current_box_id} "
            f"(ArUco ID {detected_id}): {verdict}"
        )
        self._result_pub.publish(String(data=f"{verdict}:{self._current_box_id}"))
        self._box_results[self._current_box_id] = verdict

        if verdict == "FAULT":
            self._transition(FSM.AWAITING_HUMAN)
        else:
            self._transition(FSM.IDLE)

    # ------------------------------------------------------------------ operator
    def _on_decision(self, msg: String):
        decision = msg.data.strip().upper()
        self.get_logger().info(f"Operator decision: {decision}")

        if decision == "START" and self._state == FSM.SETUP:
            self._snapshot_visit_queue()
            self._transition(FSM.IDLE)
            return

        if decision.startswith("REPAIR_ORDER:"):
            if self._state != FSM.REVIEWING_QUEUE:
                return
            order_str = decision.split(":")[1]
            if order_str == "NONE" or not order_str:
                self._transition(FSM.MISSION_COMPLETE)
            else:
                # Operator sends ArUco IDs; map each back to its stored (pos, id) entry.
                order_ids = [int(x) for x in order_str.split(",")]
                by_id = {bid: (pos, bid) for pos, bid in self._repair_queue}
                self._current_repair_batch = [by_id[i] for i in order_ids if i in by_id]
                self._repair_queue = [e for e in self._repair_queue
                                      if e[1] not in order_ids]
                self._transition(FSM.IDLE)
            return

        if self._state != FSM.AWAITING_HUMAN:
            return

        if decision == "REPAIR_NOW":
            # Bot is at scan distance and already perpendicular — creep in to dock distance.
            # Flag the repair phase so _finish_dock routes to REPAIRING, not another INSPECTING.
            self._is_repair_phase = True
            self._transition(FSM.APPROACH_DOCK)
        elif decision == "QUEUE":
            target = self._current_beacon if ARENA_FREE else self._current_box_pos
            self._repair_queue.append((target, self._current_box_id))
            self._box_results[self._current_box_id] = "QUEUED"
            self._result_pub.publish(String(data=f"QUEUED:{self._current_box_id}"))
            self._transition(FSM.IDLE)
        elif decision == "SKIP":
            self._box_results[self._current_box_id] = "SKIPPED"
            self._result_pub.publish(String(data=f"SKIPPED:{self._current_box_id}"))
            self._transition(FSM.IDLE)

    def _cancel_repair_timer(self):
        if self._repair_timer is not None:
            self._repair_timer.cancel()
            self.destroy_timer(self._repair_timer)
            self._repair_timer = None

    def _start_mock_repair(self):
        """Mock repair: from the ~10 cm dock, nudge MOCK_REPAIR_CM forward, pause, then reverse
        back to the ~10 cm gap and report REPAIRED. Pure SCADA cmd_vel motion — no reed/firmware
        REPAIR_MODE dependency. 20 Hz tick keeps the firmware watchdog fed throughout."""
        self._cancel_repair_timer()
        # distance / speed = seconds; at 20 Hz that is seconds * 20 ticks per leg
        self._mock_leg_ticks = max(1, int(round((MOCK_REPAIR_CM / 100.0) / APPROACH_SPEED * 20)))
        self._mock_phase = "forward"
        self._mock_ticks = self._mock_leg_ticks
        self.get_logger().info(
            f"Mock repair at box {self._box_label()}: nudge {MOCK_REPAIR_CM:.1f} cm forward, "
            f"pause, then back"
        )
        self._repair_timer = self.create_timer(0.05, self._mock_repair_tick)   # 20 Hz

    def _mock_repair_tick(self):
        if self._state != FSM.REPAIRING:
            self._cancel_repair_timer()
            return
        twist = Twist()
        # Hold the dock heading throughout the nudge. Open-loop straight fails: MIN_MOVE_PWM floors
        # both wheels to ~100 so any motor mismatch curves the bot into the box edge, and the reverse
        # leg never unwinds the tilt. Steer on the same box-face heading error the dock creep uses —
        # correction stays small enough that linear.x dominates (no floor-induced spin).
        steer = 0.0
        if ARENA_FREE:
            heading_err = self._heading_alignment_err()
            if heading_err is not None:
                steer = max(-0.2, min(0.2, PX_TURN_KP * heading_err * PX_TURN_SIGN))
        if self._mock_phase == "forward":
            twist.linear.x = APPROACH_SPEED
            twist.angular.z = steer
            self._cmd_vel_pub.publish(twist)
            self._mock_ticks -= 1
            if self._mock_ticks <= 0:
                self._mock_phase = "pause"
                self._mock_ticks = 10          # 0.5 s settle with motors stopped
        elif self._mock_phase == "pause":
            self._cmd_vel_pub.publish(twist)   # zero — hold still (keeps watchdog fed)
            self._mock_ticks -= 1
            if self._mock_ticks <= 0:
                self._mock_phase = "reverse"
                self._mock_ticks = self._mock_leg_ticks
        elif self._mock_phase == "reverse":
            twist.linear.x = -APPROACH_SPEED
            # angular.z is body rotation (CCW+), independent of travel sign, and the P-loop recomputes
            # heading_err from the camera each tick — so the same steer drives heading to the dock
            # heading whether creeping in or backing out. No inversion.
            twist.angular.z = steer
            self._cmd_vel_pub.publish(twist)
            self._mock_ticks -= 1
            if self._mock_ticks <= 0:
                self._cmd_vel_pub.publish(Twist())   # stop
                self._send_cmd("STOP")
                self._cancel_repair_timer()
                self._mock_phase = None
                self._finish_repair()

    def _finish_repair(self):
        """Record the mock repair as done and advance the mission (mirrors the old
        STATUS:REPAIRED path, but self-driven since there's no firmware repair cycle)."""
        self._box_results[self._current_box_id] = "REPAIRED"
        self._result_pub.publish(String(data=f"REPAIRED:{self._current_box_id}"))
        self.get_logger().info(f"Mock repair complete for box {self._box_label()} → REPAIRED")
        self._is_repair_phase = False   # back to normal inspect routing for the next box
        self._transition(FSM.IDLE)

    # ------------------------------------------------------------------ motion primitives
    def _send_primitive(self, kind: str, value: float, on_done):
        """Fire one encoder closed-loop primitive on the firmware and run `on_done` when it reports
        STATUS:<kind>_DONE. While it is in flight, SCADA publishes NO /cmd_vel — the firmware owns the
        motors for the duration — so the two never fight over the motor queue. `on_done` may be None."""
        self._prim_kind     = kind
        self._prim_done_cb  = on_done
        self._prim_pending  = True
        self._prim_deadline = time.monotonic() + PRIM_TIMEOUT_S
        self._send_cmd(f"{kind}:{value:.1f}")

    def _primitive_done(self, timed_out: bool = False):
        if not self._prim_pending:
            return
        self._prim_pending = False
        cb = self._prim_done_cb
        self._prim_done_cb = None
        if timed_out:
            self.get_logger().warn(f"primitive {self._prim_kind} timed out — continuing")
        if cb is not None:
            cb()

    def _abort_primitive(self):
        """Drop in-flight primitive bookkeeping when a phase is torn down. A STATUS:*_DONE that
        arrives afterwards is ignored (guard in _on_status), so no stale continuation fires."""
        self._prim_pending = False
        self._prim_done_cb = None

    # ------------------------------------------------------------------ stationary sampling
    def _start_sampling(self, getter, n, on_result):
        """Hold the bot still, then collect `n` values from getter() on distinct fresh camera frames
        and hand the list to on_result(). Measuring only while stopped means the overhead marker is
        sharp (no motion blur) and the median rejects the occasional dropout/flip — this is what makes
        the computed goal trustworthy enough to drive a one-shot encoder primitive."""
        self._stop_sampling()
        self._samp = {
            "getter": getter, "n": n, "on_result": on_result, "vals": [], "last_seq": -1,
            "settle_until": time.monotonic() + SETTLE_S,
            "deadline":     time.monotonic() + SETTLE_S + 4.0,
        }
        self._samp_timer = self.create_timer(0.02, self._sample_tick)

    def _sample_tick(self):
        s = self._samp
        if s is None:
            self._stop_sampling()
            return
        now = time.monotonic()
        self._cmd_vel_pub.publish(Twist())   # keep still → sharp frames, firmware watchdog fed
        if now < s["settle_until"]:
            return
        if self._bot_px is not None and self._px_seq != s["last_seq"]:
            s["last_seq"] = self._px_seq
            v = s["getter"]()
            if v is not None:
                s["vals"].append(v)
        if len(s["vals"]) >= s["n"] or now >= s["deadline"]:
            cb, vals = s["on_result"], s["vals"]
            self._stop_sampling()
            cb(vals)

    def _stop_sampling(self):
        if self._samp_timer is not None:
            self._samp_timer.cancel()
            self.destroy_timer(self._samp_timer)
            self._samp_timer = None
        self._samp = None

    # ------------------------------------------------------------------ primitive alignment
    def _start_align_prim(self):
        """Measure-while-stopped → one TURN/FWD → re-check, on encoder primitives. Ends BOTH square-on
        (heading within TURN_TOL_DEG) and centred on the box forward axis (within LAT_TOL_PX)."""
        self._stop_sampling()
        self._abort_primitive()
        self._align_round = 0
        self._align_blind = 0
        desired = self._box_desired_hdg(self._current_beacon)
        des_deg = math.degrees(desired) if desired is not None else 0.0
        self.get_logger().info(
            f"Aligning (primitives): face box {self._box_label()} "
            f"(desired hdg={des_deg:.1f}°, target ±{TURN_TOL_DEG:.1f}°)"
        )
        self._align_measure_heading()

    def _align_measure_heading(self):
        if self._state != FSM.ALIGNING:
            return
        self._start_sampling(self._heading_alignment_err, HEADING_SAMPLES, self._align_on_heading)

    def _align_on_heading(self, errs):
        if self._state != FSM.ALIGNING:
            return
        err = _circ_median(errs)
        if err is None:
            self._align_blind += 1
            if self._align_blind >= 5:
                self.get_logger().warn(
                    f"Box {self._box_label()} heading unreadable after {self._align_blind} windows "
                    f"— proceeding best-effort"
                )
                self._finish_align(0.0)
                return
            self._align_measure_heading()   # bot marker unseen this window — look again
            return
        self._align_blind = 0
        if abs(err) <= math.radians(TURN_TOL_DEG):
            self._align_measure_lat()
            return
        self._send_primitive("TURN", math.degrees(err) * PX_TURN_SIGN, self._align_measure_heading)

    def _align_measure_lat(self):
        if self._state != FSM.ALIGNING:
            return
        self._start_sampling(self._lateral_box_err, LAT_SAMPLES, self._align_on_lat)

    def _align_on_lat(self, lats):
        if self._state != FSM.ALIGNING:
            return
        lat = _median(lats) or 0.0
        if abs(lat) <= LAT_TOL_PX:
            self._finish_align(self._heading_alignment_err() or 0.0)
            return
        if self._align_round >= ALIGN_MAX_ROUNDS:
            self.get_logger().warn(
                f"Box {self._box_label()} not centred after {ALIGN_MAX_ROUNDS} rounds "
                f"(lateral {lat:.0f}px) — proceeding best-effort"
            )
            self._finish_align(self._heading_alignment_err() or 0.0)
            return
        self._align_round += 1
        self._lat_sign = 1.0 if lat > 0 else -1.0
        self.get_logger().info(
            f"Centring on box {self._box_label()}: lateral offset {lat:.0f}px "
            f"(round {self._align_round}/{ALIGN_MAX_ROUNDS})"
        )
        # Turn to face the sideways-drive heading that kills the offset, drive the measured offset
        # across (px→mm via LAT_PX_PER_MM), then return to the heading stage — which turns square-on
        # again and re-checks the offset. The re-check loop absorbs any LAT_PX_PER_MM scale error.
        tgt = self._lat_drive_hdg()
        cur = self._cur_px()
        if tgt is None or cur is None:
            self._align_measure_heading()
            return
        turn_deg = math.degrees(_wrap(tgt - cur[2])) * PX_TURN_SIGN
        drive_mm = LAT_GAIN * abs(lat) / LAT_PX_PER_MM
        self._send_primitive(
            "TURN", turn_deg,
            lambda: self._send_primitive("FWD", drive_mm, self._align_measure_heading),
        )

    # ------------------------------------------------------------------ primitive approach (scan/dock)
    def _start_approach_prim(self, target_mm: float):
        """Creep to `target_mm` TOF in straight FWD steps, re-reading the TOF between each (so TOF
        noise/slip can't accumulate). Straight steps mean the bot stays square-on and centred — no
        skid carrying the camera off the box face."""
        self._abort_primitive()
        self._approach_target   = target_mm
        self._approach_settling = False
        self._approach_fwd_mm   = 0.0
        self.get_logger().info(
            f"Approach (primitives): straight FWD → {target_mm:.0f} mm for box {self._box_label()}"
        )
        self._approach_step()

    def _approach_step(self):
        if self._state not in (FSM.APPROACH_SCAN, FSM.APPROACH_DOCK):
            return
        tof = self._tof_mm
        tof_sane = 30.0 <= tof < 2000.0

        if self._approach_settling:
            # Dock settle-back: reverse straight until the TOF shows a reliable gap >= DOCK_STOP_MM,
            # so every run ends at the same readable ~10 cm stand-off no matter how the stop triggered.
            if tof_sane and tof >= DOCK_STOP_MM:
                self._send_cmd("STOP")
                self._approach_settling = False
                self.get_logger().info(
                    f"Settled at {tof:.0f} mm — readable gap for box {self._box_label()} → "
                    f"{'repair' if self._is_repair_phase else 'inspect'}"
                )
                self._finish_dock()
            else:
                self._send_primitive("FWD", -DOCK_STEP_MM, self._approach_step)
            return

        reached = tof_sane and tof <= (self._approach_target + DOCK_ANTICIPATE_MM)
        too_close = (not tof_sane) and tof < 30.0           # saturated TOF dead zone → already in
        capped = self._approach_fwd_mm >= 400.0             # drove far without seeing the box — bail

        if reached or too_close or capped:
            self._send_cmd("STOP")
            if capped and not reached:
                self.get_logger().warn(
                    f"Approach cap hit for box {self._box_label()} (TOF {tof:.0f} mm) — proceeding"
                )
            if self._state == FSM.APPROACH_SCAN:
                self.get_logger().info(
                    f"Scan distance for box {self._box_label()} (TOF {tof:.0f} mm) → INSPECTING"
                )
                self._transition(FSM.INSPECTING)
            elif DOCK_SETTLE and not too_close:
                self.get_logger().info(
                    f"Dock reached for box {self._box_label()} (TOF {tof:.0f} mm) — settling"
                )
                self._approach_settling = True
                self._approach_step()
            elif DOCK_SETTLE:
                self._approach_settling = True
                self._approach_step()
            else:
                self._finish_dock()
            return

        togo = (tof - self._approach_target) if tof_sane else DOCK_STEP_MM
        step = min(DOCK_STEP_MM, max(2.0, togo - DOCK_ANTICIPATE_MM)) if tof_sane else DOCK_STEP_MM
        self._approach_fwd_mm += step
        self._send_primitive("FWD", step, self._approach_step)

    # ------------------------------------------------------------------ primitive inspect back-off
    def _start_inspect_backoff_prim(self):
        self._stop_inspect_backoff()
        self._backoff_done_mm = 0.0
        self._backoff_settle  = INSPECT_BACKOFF_SETTLE_TICKS
        self._backoff_hold    = INSPECT_BACKOFF_HOLD_TICKS
        self._backoff_stream_dead = False
        self._backoff_stale_logged = False
        self.get_logger().info(
            f"Inspect back-off (primitives) for box {self._box_label()}: straight FWD "
            f"-{INSPECT_BACKOFF_STEP_MM:.0f} mm steps out to {INSPECT_BACKOFF_MAX_MM:.0f} mm"
        )
        self._backoff_timer = self.create_timer(0.05, self._inspect_backoff_tick_prim)

    def _inspect_backoff_tick_prim(self):
        # Look-then-step on primitives: settle motors-off for a sharp S3 frame, then reverse one
        # straight increment. _on_aruco_detected halts this the instant a marker confirms.
        if self._state != FSM.INSPECTING:
            self._stop_inspect_backoff()
            return
        if self._prim_pending:
            return   # a back-off FWD step is running on the firmware
        if (time.monotonic() - self._last_cam_alive) > INSPECT_STREAM_STALE_S:
            self._backoff_stream_dead = True
            if not self._backoff_stale_logged:
                self.get_logger().warn(
                    f"Inspection paused for box {self._box_label()} — S3 stream stale "
                    f"(no frames >{INSPECT_STREAM_STALE_S:.0f}s); holding until it recovers"
                )
                self._backoff_stale_logged = True
            self._cmd_vel_pub.publish(Twist())
            return
        self._backoff_stale_logged = False
        if self._backoff_settle > 0:
            self._backoff_settle -= 1
            self._cmd_vel_pub.publish(Twist())
            return
        if self._backoff_done_mm >= INSPECT_BACKOFF_MAX_MM:
            if self._backoff_hold > 0:
                self._backoff_hold -= 1
                self._cmd_vel_pub.publish(Twist())
                return
            self._inspect_miss("backed off through readable band + grace, no marker")
            return
        self._backoff_done_mm += INSPECT_BACKOFF_STEP_MM
        self._backoff_settle = INSPECT_BACKOFF_SETTLE_TICKS
        self._send_primitive("FWD", -INSPECT_BACKOFF_STEP_MM, None)

    # ------------------------------------------------------------------ primitive mock repair
    def _start_mock_repair_prim(self):
        self._cancel_repair_timer()
        self._mock_stage = "forward"
        # Nose right up to the box off the live TOF gap (leave REPAIR_CONTACT_MM clearance), rather
        # than a fixed nudge that stops well short. Reverse the SAME distance afterward so the bot
        # ends back at the docking stand-off. Fall back to the fixed nudge if the TOF is unreadable.
        tof = self._tof_mm
        tof_sane = tof is not None and 0 < tof < 2000
        if tof_sane:
            self._repair_fwd_mm = max(10.0, tof - REPAIR_CONTACT_MM)
        else:
            self._repair_fwd_mm = MOCK_REPAIR_MM
        self.get_logger().info(
            f"Mock repair (primitives) at box {self._box_label()}: nose in {self._repair_fwd_mm:.0f} mm "
            f"(TOF {tof if tof_sane else 'n/a'}) in {DOCK_STEP_MM:.0f} mm steps, pause, then exact reverse"
        )
        # Nose in / back out in small DOCK_STEP_MM chunks rather than one big FWD: a long single
        # primitive saturates the P-term to max PWM and launches hard (the jerk). Short steps stay in
        # the gentle P-region so the bot creeps up to the box and backs off smoothly.
        self._repair_remaining = self._repair_fwd_mm
        self._mock_repair_forward_step()

    def _mock_repair_forward_step(self):
        if self._state != FSM.REPAIRING:
            return
        if self._repair_remaining <= 0.5:
            self._mock_repair_after_forward()
            return
        step = min(DOCK_STEP_MM, self._repair_remaining)
        self._repair_remaining -= step
        self._send_primitive("FWD", step, self._mock_repair_forward_step)

    def _mock_repair_after_forward(self):
        if self._state != FSM.REPAIRING:
            return
        self._cancel_repair_timer()
        self._repair_timer = self.create_timer(0.5, self._mock_repair_reverse)   # brief pause

    def _mock_repair_reverse(self):
        self._cancel_repair_timer()
        if self._state != FSM.REPAIRING:
            return
        self._repair_remaining = self._repair_fwd_mm
        self._mock_repair_reverse_step()

    def _mock_repair_reverse_step(self):
        if self._state != FSM.REPAIRING:
            return
        if self._repair_remaining <= 0.5:
            self._mock_repair_done()
            return
        step = min(DOCK_STEP_MM, self._repair_remaining)
        self._repair_remaining -= step
        self._send_primitive("FWD", -step, self._mock_repair_reverse_step)

    def _mock_repair_done(self):
        if self._state != FSM.REPAIRING:
            return
        self._send_cmd("STOP")
        self._mock_stage = None
        self._finish_repair()

    def _on_status(self, msg: String):
        status = msg.data.strip()
        if status in ("STATUS:TURN_DONE", "STATUS:FWD_DONE"):
            self._primitive_done()
            return
        self.get_logger().info(f"Bot status: {status}")

        if status == "STATUS:REPAIRED" and self._state == FSM.REPAIRING:
            self._cancel_repair_timer()
            self._box_results[self._current_box_id] = "REPAIRED"
            self._result_pub.publish(
                String(data=f"REPAIRED:{self._current_box_id}")
            )
            self._transition(FSM.IDLE)

    # ------------------------------------------------------------------ watchdog
    def _watchdog_tick(self):
        # A primitive whose STATUS:*_DONE never arrived (lost packet / firmware safety timeout) would
        # otherwise wedge the FSM — resume its continuation once the deadline passes.
        if self._prim_pending and time.monotonic() > self._prim_deadline:
            self._primitive_done(timed_out=True)
        if self._state == FSM.AWAITING_HUMAN:
            self._send_cmd("STOP")

    # ------------------------------------------------------------------ helpers
    def _send_cmd(self, cmd: str):
        self._cmd_pub.publish(String(data=cmd))
        self.get_logger().debug(f"→ bot: {cmd!r}")


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
