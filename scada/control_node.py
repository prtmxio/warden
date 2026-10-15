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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Range
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

try:
    from nav2_msgs.action import NavigateToPose
    HAS_NAV2 = True
except ImportError:
    HAS_NAV2 = False

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import (
    BOX_ARUCO_IDS, BOX_POSITIONS, fault_config,
    SCAN_STOP_MM, DOCK_STOP_MM, NAV_STANDOFF_M,
    PERP_TOL_DEG, LATERAL_TOL_M, APPROACH_SPEED,
    APPROACH_YAW_KP, ALIGN_YAW_KP,
    CAM_FWD, CAM_LAT, AXLE_FWD, AXLE_LAT,
    WATCHDOG_MS,
)

WATCHDOG_INTERVAL = WATCHDOG_MS / 1000.0 * 0.8   # send STOP slightly faster than 500 ms

# Boxes line the north wall and are always approached from the south, so the perpendicular
# (full-view) heading is a constant: face +Y.
TARGET_YAW = math.pi / 2.0


def _wrap(a: float) -> float:
    """Wrap an angle to (-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


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
        # Closed-loop align/approach runs in the odom≡map frame (same frame Nav2 and /cmd_vel
        # use). /robot_pose (overhead ArUco) is uncalibrated relative to it, so use /odom here.
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        # Nav2 action client (only wired up from Day 7 onward)
        if HAS_NAV2:
            self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # --- state ---
        self._state          = FSM.SETUP
        self._current_box_id = None         # aruco ID of box being inspected
        self._visit_queue    = list(BOX_ARUCO_IDS)   # boxes left to visit
        self._repair_queue   = []            # fault boxes queued for later repair
        self._is_repair_phase = False        # true when executing queued repairs
        self._current_repair_batch = []      # list of boxes to repair in current batch
        self._tof_mm         = 9999.0
        self._box_results    = {}            # aruco_id → "FAULT"/"OK"/"REPAIRED"/"SKIP"
        self._pose           = None          # (x, y, yaw) from /odom, or None
        self._align_timer    = None          # active only during ALIGNING
        self._align_phase    = "center_turn" # center_turn → center_drive → perp_turn
        self._align_ok_count = 0             # consecutive in-tolerance perp ticks
        self._inspect_retries = 0            # failed full-view reads for current box
        self._inspect_timer  = None          # 5 s timeout waiting for ArUco from bot_cam
        self._repair_timer   = None          # 2.5 s → sends REPAIR_STOP to firmware

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
            self._navigate_to_current_box()

        elif new_state == FSM.ALIGNING:
            self._start_align()

        elif new_state == FSM.APPROACH_SCAN:
            self.get_logger().info(f"Approach-scan: TOF creep to {SCAN_STOP_MM:.0f} mm")

        elif new_state == FSM.APPROACH_DOCK:
            self.get_logger().info(f"Approach-dock: TOF creep to {DOCK_STOP_MM:.0f} mm")

        elif new_state == FSM.INSPECTING:
            self._start_inspection()

        elif new_state == FSM.REPAIRING:
            self._send_cmd("REPAIR_MODE")
            self._repair_timer = self.create_timer(2.5, self._send_repair_stop)

        elif new_state == FSM.MISSION_COMPLETE:
            self.get_logger().info("=== MISSION COMPLETE ===")
            self.get_logger().info(f"Results: {self._box_results}")

    def _maybe_start_next_box(self):
        """Pick the next unvisited box or finish the mission."""
        if self._visit_queue:
            self._current_box_id = self._visit_queue.pop(0)
            self.get_logger().info(f"Next target: box ArUco ID {self._current_box_id}")
            self._transition(FSM.NAVIGATING)
            return

        if self._current_repair_batch:
            self._is_repair_phase = True
            self._current_box_id = self._current_repair_batch.pop(0)
            if self._current_box_id in self._repair_queue:
                self._repair_queue.remove(self._current_box_id)
            self.get_logger().info(f"Next target (REPAIR): box ArUco ID {self._current_box_id}")
            self._transition(FSM.NAVIGATING)
            return

        if self._repair_queue:
            self._transition(FSM.REVIEWING_QUEUE)
            return

        self._transition(FSM.MISSION_COMPLETE)

    # ------------------------------------------------------------------ navigation
    def _navigate_to_current_box(self):
        pos = BOX_POSITIONS.get(self._current_box_id)
        if pos is None:
            self.get_logger().error(f"No position for box {self._current_box_id}")
            threading.Timer(1.0, self._on_nav_arrived).start()
            return

        if HAS_NAV2 and self._nav_client.wait_for_server(timeout_sec=1.0):
            # Pre-docking stop point: 20 cm south of the box
            # (all boxes are near the north wall, bot approaches from south)
            from nav2_msgs.action import NavigateToPose
            from geometry_msgs.msg import PoseStamped
            import math as _m

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = pos[0]
            # Pre-dock standoff south of the box; control_node fine-aligns + creeps from here
            goal_msg.pose.pose.position.y = pos[1] - NAV_STANDOFF_M
            # Face north (+Y) to approach the box
            goal_msg.pose.pose.orientation.z = _m.sin(_m.pi / 4)
            goal_msg.pose.pose.orientation.w = _m.cos(_m.pi / 4)

            self.get_logger().info(
                f"Nav2 goal → box {self._current_box_id} pre-dock at "
                f"({goal_msg.pose.pose.position.x:.2f}, "
                f"{goal_msg.pose.pose.position.y:.2f})"
            )
            future = self._nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self._on_nav_goal_response)
        else:
            self.get_logger().info(
                f"Nav2 unavailable — simulating 3 s travel to box {self._current_box_id}"
            )
            threading.Timer(3.0, self._on_nav_arrived).start()

    def _on_nav_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected goal — falling back to timer")
            threading.Timer(3.0, self._on_nav_arrived).start()
            return
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        from action_msgs.msg import GoalStatus
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 goal reached")
            self._on_nav_arrived()
        else:
            self.get_logger().warn(
                f"Nav2 goal failed (status {status}) — bot near target, proceeding to dock"
            )
            self._on_nav_arrived()

    def _on_nav_arrived(self):
        if self._state == FSM.NAVIGATING:
            self._transition(FSM.ALIGNING)

    # ------------------------------------------------------------------ pose
    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            _wrap(yaw),
        )

    # ------------------------------------------------------------------ alignment
    def _start_align(self):
        """Center the wheel-axle (rotation centre) on the box centerline, then face the box.
        Centering the axle — not base_footprint — makes the camera land centred and is
        rotation-invariant, so the final perp-turn doesn't push it back off."""
        self._stop_align_timer()   # guard against a stale timer on retry
        self._align_ok_count = 0
        pos = BOX_POSITIONS.get(self._current_box_id)
        need_center = (self._pose is not None and pos is not None
                       and abs(pos[0] - _axle_x(self._pose[0], self._pose[2])) > LATERAL_TOL_M)
        self._align_phase = "center_turn" if need_center else "perp_turn"
        self.get_logger().info(
            f"Aligning (phase={self._align_phase}, target ±{PERP_TOL_DEG:.1f}°)"
        )
        self._align_timer = self.create_timer(0.05, self._align_tick)   # 20 Hz

    def _align_tick(self):
        if self._state != FSM.ALIGNING:
            self._stop_align_timer()
            return
        if self._pose is None:
            return
        pos = BOX_POSITIONS.get(self._current_box_id)
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
        if self._state not in (FSM.APPROACH_SCAN, FSM.APPROACH_DOCK):
            return

        target_mm = SCAN_STOP_MM if self._state == FSM.APPROACH_SCAN else DOCK_STOP_MM
        twist = Twist()
        if self._tof_mm <= target_mm:
            self._cmd_vel_pub.publish(twist)   # stop
            self._send_cmd("STOP")
            if self._state == FSM.APPROACH_SCAN:
                self._transition(FSM.INSPECTING)
            else:
                self._transition(FSM.REPAIRING)
        else:
            twist.linear.x = APPROACH_SPEED
            # Hold perpendicular heading while creeping so the QR stays square-on and centred
            # (lateral centering was done in ALIGNING via the axle).
            if self._pose is not None:
                yaw_err = _wrap(TARGET_YAW - self._pose[2])
                twist.angular.z = max(-0.3, min(0.3, APPROACH_YAW_KP * yaw_err))
            self._cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------ inspection
    def _start_inspection(self):
        self._send_cmd("SCAN")
        self.get_logger().info(
            f"SCAN sent for box {self._current_box_id} — "
            f"waiting for ArUco detection from bot_cam (5 s timeout)"
        )
        self._inspect_timer = self.create_timer(5.0, self._inspect_timeout)

    def _cancel_inspect_timer(self):
        if self._inspect_timer is not None:
            self._inspect_timer.cancel()
            self.destroy_timer(self._inspect_timer)
            self._inspect_timer = None

    def _on_aruco_detected(self, msg: String):
        if self._state != FSM.INSPECTING:
            return
        try:
            detected_id = int(msg.data)
        except ValueError:
            return
        if detected_id not in BOX_ARUCO_IDS:
            return
        self._cancel_inspect_timer()
        self._resolve_inspection(detected_id)

    def _inspect_timeout(self):
        self._cancel_inspect_timer()
        if self._state != FSM.INSPECTING:
            return
        self._inspect_retries += 1
        if self._inspect_retries <= 2:
            self.get_logger().warn(
                f"ArUco timeout for box {self._current_box_id} "
                f"(retry {self._inspect_retries}/2) — re-aligning"
            )
            self._transition(FSM.ALIGNING)
            return
        self.get_logger().warn(
            f"ArUco still not detected for box {self._current_box_id} — treating as OK"
        )
        self._inspect_retries = 0
        self._box_results[self._current_box_id] = "OK"
        self._result_pub.publish(String(data=f"OK:{self._current_box_id}"))
        self._transition(FSM.IDLE)

    def _scan_view_valid(self) -> bool:
        """True only if the bot is square-on (±PERP_TOL_DEG), centred on the box
        (±LATERAL_TOL_M) and within scan range — i.e. the whole QR is readable."""
        if self._pose is None:
            return False
        pos = BOX_POSITIONS.get(self._current_box_id)
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
                    f"{self._current_box_id} (retry {self._inspect_retries}/2) — re-aligning"
                )
                self._transition(FSM.ALIGNING)
                return
            self.get_logger().warn(
                f"View geometry still invalid for box {self._current_box_id} — treating as OK"
            )
            self._inspect_retries = 0
            self._box_results[self._current_box_id] = "OK"
            self._result_pub.publish(String(data=f"OK:{self._current_box_id}"))
            self._transition(FSM.IDLE)
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
            self._transition(FSM.IDLE)
            return

        if decision.startswith("REPAIR_ORDER:"):
            if self._state != FSM.REVIEWING_QUEUE:
                return
            order_str = decision.split(":")[1]
            if order_str == "NONE" or not order_str:
                self._transition(FSM.MISSION_COMPLETE)
            else:
                self._current_repair_batch = [int(x) for x in order_str.split(",")]
                self._transition(FSM.IDLE)
            return

        if self._state != FSM.AWAITING_HUMAN:
            return

        if decision == "REPAIR_NOW":
            # Bot is at scan distance and already perpendicular — creep in to dock distance
            self._transition(FSM.APPROACH_DOCK)
        elif decision == "QUEUE":
            self._repair_queue.append(self._current_box_id)
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

    def _send_repair_stop(self):
        self._cancel_repair_timer()
        if self._state == FSM.REPAIRING:
            self._send_cmd("REPAIR_STOP")

    def _on_status(self, msg: String):
        status = msg.data.strip()
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
