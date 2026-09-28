"""
sim_inspection_node — Tier 1 Gazebo simulation support.

Replaces udp_bridge_node + mock_bot + physical sensors for sim runs.

Publishes:
  /map           OccupancyGrid  — static arena with walls + box footprints
  /tof_range     Range          — geometric distance from bot to box face during the
                                  approach/inspect states; 999 mm (far) in all other states
  /bot_status    String         — STATUS:REPAIRED after repair sequence
  /cmd_vel       Twist          — backward motion for 2 s after repair (simulates firmware
                                  reed-triggered reversal)

Subscribes:
  /fsm_state     String         — to know when the bot is approaching / REPAIRING
  /bot_command   String         — REPAIR_MODE → STATUS:REPAIRED after 2s + reversal
  /odom          Odometry       — bot pose (odom≡map) for the geometric TOF beam

Note: /robot_pose is owned by sim_vision_node (real ArUco detection). The Gazebo depth
sensor stays in the world for visualization but is no longer used for range — a narrow-beam
VL53L0X is synthesised geometrically from /odom + box positions (robust to sensor mounting).
"""

import math
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Range
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import BOX_POSITIONS, BOX_ARUCO_IDS, ARENA_SIZE_CM

# Arena is ARENA_SIZE_CM × ARENA_SIZE_CM, resolution 5 cm/cell
_RES   = 0.05           # m per cell
_COLS  = int(ARENA_SIZE_CM / 100.0 / _RES) + 2   # +2 for walls
_ROWS  = _COLS
_ORIG  = -_RES          # origin so cell (1,1) = (0,0) in world

FAKE_TOF_FAR_MM = 999.0   # reported outside the approach/inspect states

# Narrow-beam VL53L0X emulation: the physical sensor sits this far ahead of the odom
# origin (TOF_joint x in warden.urdf.xacro); box south face is BOX_HALF south of centre.
SENSOR_OFFSET = 0.20      # m — sensor forward of base_footprint/odom origin
BOX_HALF      = 0.05      # m — half a 10 cm box cube
_TOF_STATES   = ("APPROACH_SCAN", "APPROACH_DOCK", "INSPECTING")


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
        # Bot pose (odom≡map) to synthesise a narrow-beam TOF geometrically
        self.create_subscription(Odometry, "/odom",        self._on_odom, 10)

        self._fsm_state  = "SETUP"
        self._bot        = None          # (x, y, yaw) from /odom, or None
        self._reverse_ticks = 0         # >0 while post-repair reversal is active

        self._map_data = _build_arena_map()
        self.create_timer(2.0,  self._pub_map)   # republish /map every 2s (Nav2 may start late)
        self.create_timer(0.10, self._pub_tof)   # 10 Hz
        self.create_timer(0.10, self._reverse_tick)  # drives post-repair reversal

        self._pub_map()
        self.get_logger().info("sim_inspection_node ready — geometric TOF active")

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
        
    # ── pose ───────────────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._bot = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    # ── TOF ───────────────────────────────────────────────────────────────────

    def _geometric_tof_m(self) -> float:
        """Narrow-beam range from the forward-mounted sensor to the south face of the box
        the bot is lined up with (all boxes on the north wall, approached facing +Y)."""
        if self._bot is None:
            return FAKE_TOF_FAR_MM / 1000.0
        x, y, yaw = self._bot
        sx = x + SENSOR_OFFSET * math.cos(yaw)
        sy = y + SENSOR_OFFSET * math.sin(yaw)
        # Pick the box the bot is most laterally aligned with, measure the forward gap to its face
        bx, by = min(BOX_POSITIONS.values(), key=lambda p: abs(sx - p[0]))
        return max(0.0, (by - BOX_HALF) - sy)

    def _pub_tof(self):
        tof_m = (self._geometric_tof_m()
                 if self._fsm_state in _TOF_STATES
                 else FAKE_TOF_FAR_MM / 1000.0)
        msg = Range()
        msg.header.stamp      = self.get_clock().now().to_msg()
        msg.header.frame_id   = "tof_sensor"
        msg.radiation_type    = Range.INFRARED
        msg.field_of_view     = 0.1
        msg.min_range         = 0.05
        msg.max_range         = 5.0
        msg.range             = tof_m
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
