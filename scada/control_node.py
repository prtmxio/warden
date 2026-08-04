"""
control_node — mission FSM + Nav2 action client.

States (in order):
  SETUP           → operator configures box positions in GUI
  IDLE            → waiting for START RUN
  NAVIGATING      → Nav2 driving to next box
  DOCKING         → TOF-guided final approach (<DOCKING_PHASE_CM)
  INSPECTING      → bot_cam scanning box side marker
  AWAITING_HUMAN  → fault detected, waiting for operator decision
  REPAIRING       → bot executing physical repair sequence
  RETURNING_HOME  → Nav2 driving back to start pose
  MISSION_COMPLETE

Subscribes:
  /tof_range          Range
  /bot_status         String   (STATUS:REPAIRED, STATUS:REED, …)
  /operator_decision  String   (REPAIR_NOW / QUEUE / SKIP)
  /robot_pose         PoseWithCovarianceStamped  (not used until Day 7)

Publishes:
  /bot_command        String   (SCAN / REPAIR_MODE / STOP)
  /inspection_result  String   (FAULT:<id> / OK:<id>)

Watchdog: publishes STOP to /bot_command every 400 ms while in AWAITING_HUMAN.
"""

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Range
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

try:
    from nav2_msgs.action import NavigateToPose
    HAS_NAV2 = True
except ImportError:
    HAS_NAV2 = False

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import (
    BOX_ARUCO_IDS, BOX_POSITIONS, fault_config,
    DOCKING_STOP_MM, DOCKING_PHASE_CM,
    WATCHDOG_MS,
)

WATCHDOG_INTERVAL = WATCHDOG_MS / 1000.0 * 0.8   # send STOP slightly faster than 500 ms


class FSM:
    SETUP           = "SETUP"
    IDLE            = "IDLE"
    NAVIGATING      = "NAVIGATING"
    DOCKING         = "DOCKING"
    INSPECTING      = "INSPECTING"
    AWAITING_HUMAN  = "AWAITING_HUMAN"
    REPAIRING       = "REPAIRING"
    RETURNING_HOME  = "RETURNING_HOME"
    MISSION_COMPLETE = "MISSION_COMPLETE"


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # --- publishers ---
        self._cmd_pub    = self.create_publisher(String, "/bot_command",      10)
        self._result_pub = self.create_publisher(String, "/inspection_result", 10)

        # --- subscribers ---
        self.create_subscription(Range,  "/tof_range",          self._on_tof,      10)
        self.create_subscription(String, "/bot_status",         self._on_status,   10)
        self.create_subscription(String, "/operator_decision",  self._on_decision, 10)

        # Nav2 action client (only wired up from Day 7 onward)
        if HAS_NAV2:
            self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # --- state ---
        self._state          = FSM.SETUP
        self._current_box_id = None         # aruco ID of box being inspected
        self._visit_queue    = list(BOX_ARUCO_IDS)   # boxes left to visit
        self._repair_queue   = []            # fault boxes queued for later repair
        self._tof_mm         = 9999.0
        self._box_results    = {}            # aruco_id → "FAULT"/"OK"/"REPAIRED"/"SKIP"

        # watchdog timer fires every 400 ms; only sends STOP during AWAITING_HUMAN
        self._watchdog_timer = self.create_timer(WATCHDOG_INTERVAL, self._watchdog_tick)

        self._transition(FSM.IDLE)          # auto-advance past SETUP for now (GUI handles SETUP)

    # ------------------------------------------------------------------ FSM
    def _transition(self, new_state: str):
        old = self._state
        self._state = new_state
        ts = time.strftime("%H:%M:%S")
        self.get_logger().info(f"[{ts}] FSM: {old} → {new_state}")

        if new_state == FSM.IDLE:
            self._maybe_start_next_box()

        elif new_state == FSM.NAVIGATING:
            self._navigate_to_current_box()

        elif new_state == FSM.DOCKING:
            self._start_docking()

        elif new_state == FSM.INSPECTING:
            self._start_inspection()

        elif new_state == FSM.REPAIRING:
            self._send_cmd("REPAIR_MODE")

        elif new_state == FSM.MISSION_COMPLETE:
            self.get_logger().info("=== MISSION COMPLETE ===")
            self.get_logger().info(f"Results: {self._box_results}")

    def _maybe_start_next_box(self):
        """Pick the next unvisited box or finish the mission."""
        if not self._visit_queue:
            if self._repair_queue:
                self.get_logger().info(
                    f"All boxes visited. Repair queue: {self._repair_queue}"
                )
                # GUI will handle REPAIR_QUEUE flow; for now log and complete
            self._transition(FSM.MISSION_COMPLETE)
            return
        self._current_box_id = self._visit_queue.pop(0)
        self.get_logger().info(f"Next target: box ArUco ID {self._current_box_id}")
        self._transition(FSM.NAVIGATING)

    # ------------------------------------------------------------------ navigation (stub until Day 7)
    def _navigate_to_current_box(self):
        pos = BOX_POSITIONS.get(self._current_box_id)
        self.get_logger().info(
            f"Navigating to box {self._current_box_id} at {pos} "
            f"(Nav2 not yet active — simulating arrival in 3 s)"
        )
        # Simulated travel: real Nav2 goal will replace this on Day 7
        threading.Timer(3.0, lambda: self._on_nav_arrived()).start()

    def _on_nav_arrived(self):
        """Called when Nav2 goal succeeds (or simulated timer fires)."""
        if self._state == FSM.NAVIGATING:
            self._transition(FSM.DOCKING)

    # ------------------------------------------------------------------ docking
    def _start_docking(self):
        self.get_logger().info(
            f"Docking: TOF-guided approach. Stop at {DOCKING_STOP_MM} mm."
        )
        # TOF callbacks drive the transition to INSPECTING

    def _on_tof(self, msg: Range):
        self._tof_mm = msg.range * 1000.0
        if self._state == FSM.DOCKING:
            if self._tof_mm <= DOCKING_STOP_MM:
                self._send_cmd("STOP")
                self._transition(FSM.INSPECTING)

    # ------------------------------------------------------------------ inspection
    def _start_inspection(self):
        self._send_cmd("SCAN")
        self.get_logger().info(
            f"SCAN sent for box {self._current_box_id} — "
            f"waiting for bot_cam result (simulated via fault_config)"
        )
        # In real flow, bot_cam.py publishes /inspection_result.
        # For Day 3 mock: resolve immediately from fault_config.
        threading.Timer(1.0, self._sim_inspect_result).start()

    def _sim_inspect_result(self):
        if self._state != FSM.INSPECTING:
            return
        verdict = fault_config.get(self._current_box_id, "OK")
        self.get_logger().info(
            f"Inspection result for box {self._current_box_id}: {verdict}"
        )
        self._result_pub.publish(String(data=f"{verdict}:{self._current_box_id}"))
        self._box_results[self._current_box_id] = verdict

        if verdict == "FAULT":
            self._transition(FSM.AWAITING_HUMAN)
        else:
            # OK → go to next box directly
            self._transition(FSM.IDLE)

    # ------------------------------------------------------------------ operator
    def _on_decision(self, msg: String):
        if self._state != FSM.AWAITING_HUMAN:
            return
        decision = msg.data.strip().upper()
        self.get_logger().info(f"Operator decision: {decision}")

        if decision == "REPAIR_NOW":
            self._transition(FSM.REPAIRING)
        elif decision == "QUEUE":
            self._repair_queue.append(self._current_box_id)
            self._box_results[self._current_box_id] = "QUEUED"
            self._transition(FSM.IDLE)
        elif decision == "SKIP":
            self._box_results[self._current_box_id] = "SKIPPED"
            self._transition(FSM.IDLE)

    def _on_status(self, msg: String):
        status = msg.data.strip()
        self.get_logger().info(f"Bot status: {status}")

        if status == "STATUS:REPAIRED" and self._state == FSM.REPAIRING:
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
    rclpy.shutdown()


if __name__ == "__main__":
    main()
