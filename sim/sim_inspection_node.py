"""
sim_inspection_node — Tier 1 Gazebo simulation support.

Replaces udp_bridge_node + mock_bot + physical sensors for sim runs.

Publishes:
  /map           OccupancyGrid  — static arena with walls + box footprints
  /tof_range     Range          — geometric distance from bot to box face during DOCKING;
                                  999 mm (far) in all other states
  /bot_status    String         — STATUS:REPAIRED after repair sequence
  /cmd_vel       Twist          — backward motion for 2 s after repair (simulates firmware
                                  reed-triggered reversal)

Subscribes:
  /fsm_state     String         — to know when DOCKING / REPAIRING starts
  /bot_command   String         — REPAIR_MODE → STATUS:REPAIRED after 2s + reversal
  /tof_sensor/depth_sensor/depth/image_raw  Image — real simulated depth sensor

Note: /robot_pose is owned by sim_vision_node (real ArUco detection).
"""

import math
import sys
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Range, Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import BOX_POSITIONS, BOX_ARUCO_IDS, ARENA_SIZE_CM, DOCKING_STOP_MM

# Arena is ARENA_SIZE_CM × ARENA_SIZE_CM, resolution 5 cm/cell
_RES   = 0.05           # m per cell
_COLS  = int(ARENA_SIZE_CM / 100.0 / _RES) + 2   # +2 for walls
_ROWS  = _COLS
_ORIG  = -_RES          # origin so cell (1,1) = (0,0) in world

FAKE_TOF_FAR_MM = 999.0   # reported outside DOCKING state


def _build_arena_map() -> list:
    """OccupancyGrid data: walls=100, box footprints=100, free=0."""
    data = [0] * (_ROWS * _COLS)

    # Perimeter walls
    for c in range(_COLS):
        data[0 * _COLS + c]           = 100
        data[(_ROWS - 1) * _COLS + c] = 100
    for r in range(_ROWS):
        data[r * _COLS + 0]           = 100
        data[r * _COLS + (_COLS - 1)] = 100

    # Box footprints (10 cm × 10 cm = 2×2 cells at 5 cm/cell)
    for bx_m, by_m in BOX_POSITIONS.values():
        col = int((bx_m - _ORIG) / _RES)
        row = int((by_m - _ORIG) / _RES)
        for dr in range(-1, 2):
            for dc in range(-1, 2):
                r, c = row + dr, col + dc
                if 0 <= r < _ROWS and 0 <= c < _COLS:
                    data[r * _COLS + c] = 100

    return data


class SimInspectionNode(Node):
    def __init__(self):
        super().__init__("sim_inspection_node")

        # Latched QoS for /map (Nav2 subscribes with transient_local)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._map_pub    = self.create_publisher(OccupancyGrid, "/map",        map_qos)
        self._tof_pub    = self.create_publisher(Range,         "/tof_range",  10)
        self._status_pub = self.create_publisher(String,        "/bot_status", 10)
        self._cmd_vel_pub = self.create_publisher(Twist,        "/cmd_vel",    10)

        self.create_subscription(String,   "/fsm_state",   self._on_fsm,  10)
        self.create_subscription(String,   "/bot_command", self._on_cmd,  10)
        
        # Real simulated depth sensor!
        self.create_subscription(Image, "/tof_sensor/depth_sensor/depth/image_raw", self._on_depth, 10)

        self._fsm_state  = "SETUP"
        self._tof_m      = FAKE_TOF_FAR_MM / 1000.0
        self._reverse_ticks = 0         # >0 while post-repair reversal is active

        self._map_data = _build_arena_map()
        self.create_timer(2.0,  self._pub_map)   # republish /map every 2s (Nav2 may start late)
        self.create_timer(0.10, self._pub_tof)   # 10 Hz
        self.create_timer(0.10, self._reverse_tick)  # drives post-repair reversal

        self._pub_map()
        self.get_logger().info("sim_inspection_node ready — real simulated TOF active")

    # ── map ──────────────────────────────────────────────────────────────────

    def _pub_map(self):
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = _RES
        msg.info.width      = _COLS
        msg.info.height     = _ROWS
        msg.info.origin.position.x = _ORIG
        msg.info.origin.position.y = _ORIG
        msg.info.origin.orientation.w = 1.0
        msg.data = self._map_data
        self._map_pub.publish(msg)
        
    # ── Depth Sensor parsing ──────────────────────────────────────────────────
    
    def _on_depth(self, msg: Image):
        if self._fsm_state != "DOCKING":
            self._tof_m = FAKE_TOF_FAR_MM / 1000.0
            return

        try:
            if msg.encoding == '32FC1':
                arr = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
                valid = arr[(arr > 0.05) & np.isfinite(arr) & (arr < 5.0)]
            elif msg.encoding == '16UC1':
                raw = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width)
                arr = raw.astype(np.float32) / 1000.0
                valid = arr[(raw > 0) & (arr > 0.05) & (arr < 5.0)]
            else:
                self._tof_m = FAKE_TOF_FAR_MM / 1000.0
                return

            self._tof_m = float(valid.min()) if len(valid) > 0 else FAKE_TOF_FAR_MM / 1000.0
        except Exception as e:
            self.get_logger().error(f"Error parsing depth image: {e}")
            self._tof_m = FAKE_TOF_FAR_MM / 1000.0

    # ── TOF ───────────────────────────────────────────────────────────────────

    def _pub_tof(self):
        msg = Range()
        msg.header.stamp      = self.get_clock().now().to_msg()
        msg.header.frame_id   = "tof_sensor"
        msg.radiation_type    = Range.INFRARED
        msg.field_of_view     = 0.1
        msg.min_range         = 0.05
        msg.max_range         = 5.0
        msg.range             = self._tof_m
        self._tof_pub.publish(msg)

    # ── FSM state changes ─────────────────────────────────────────────────────

    def _on_fsm(self, msg: String):
        self._fsm_state = msg.data.strip()

    # ── REPAIR_MODE → REPAIRED + reversal ────────────────────────────────────

    def _on_cmd(self, msg: String):
        if msg.data.strip() == "REPAIR_MODE":
            self.get_logger().info("sim: REPAIR_MODE received — REPAIRED in 2s then reversing")
            threading.Timer(2.0, self._finish_repair).start()

    def _finish_repair(self):
        self.get_logger().info("sim: starting 2s reversal before sending STATUS:REPAIRED")
        # Arm the reversal: 20 ticks × 0.1 s = 2 s at -0.08 m/s ≈ 0.16 m back
        self._reverse_ticks = 20

    def _reverse_tick(self):
        if self._reverse_ticks <= 0:
            return
        twist = Twist()
        self._reverse_ticks -= 1
        if self._reverse_ticks > 0:
            twist.linear.x = -0.08
        # publishes zero twist on the final tick to stop the bot
        self._cmd_vel_pub.publish(twist)
        if self._reverse_ticks == 0:
            self.get_logger().info("sim: reversal complete — sending STATUS:REPAIRED")
            self._status_pub.publish(String(data="STATUS:REPAIRED"))


def main(args=None):
    rclpy.init(args=args)
    node = SimInspectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
