"""
udp_bridge_node — ROS2 ↔ UDP ↔ ESP32-C6 relay.

Subscribes:
  /cmd_vel        Twist   → encodes as "LEFT:x RIGHT:y\n" and sends to bot UDP
  /bot_command    String  → forwards raw string to bot UDP

Publishes:
  /odom           Odometry                — from "ODOM:dl:dr:heading\n" packets
  /tof_range      Range                   — from "TOF:mm\n" packets
  /bot_status     String                  — from "STATUS:…\n" packets

UDP receive port: SCADA_LISTEN_PORT (4211)
UDP send:         BOT_IP : BOT_PORT  (4210)
"""

import math
import socket
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import String

sys.path.insert(0, "/home/xio/Desktop/warden/scada")
from arena_config import (
    BOT_IP, BOT_PORT, SCADA_LISTEN_PORT,
    WHEEL_SEPARATION, MAX_LINEAR_VEL, MAX_PWM,
)


def vel_to_pwm(linear_x: float, angular_z: float) -> tuple[int, int]:
    """Convert Twist to left/right PWM values in [-MAX_PWM, MAX_PWM]."""
    v_left  = linear_x - angular_z * WHEEL_SEPARATION / 2.0
    v_right = linear_x + angular_z * WHEEL_SEPARATION / 2.0
    scale   = MAX_PWM / MAX_LINEAR_VEL
    left    = int(max(-MAX_PWM, min(MAX_PWM, v_left  * scale)))
    right   = int(max(-MAX_PWM, min(MAX_PWM, v_right * scale)))
    return left, right


class UdpBridgeNode(Node):
    def __init__(self):
        super().__init__("udp_bridge_node")

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", SCADA_LISTEN_PORT))
        self._sock.settimeout(0.5)
        self._bot_addr = (BOT_IP, BOT_PORT)

        self._odom_pub  = self.create_publisher(Odometry, "/odom",       10)
        self._tof_pub   = self.create_publisher(Range,    "/tof_range",   10)
        self._status_pub = self.create_publisher(String,  "/bot_status",  10)

        self.create_subscription(Twist,  "/cmd_vel",     self._on_cmd_vel,    10)
        self.create_subscription(String, "/bot_command", self._on_bot_command, 10)

        # running odometry state
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info(
            f"udp_bridge: listening :{SCADA_LISTEN_PORT}, bot={self._bot_addr}"
        )

    # ------------------------------------------------------------------ send
    def _send(self, payload: str):
        try:
            self._sock.sendto(payload.encode(), self._bot_addr)
        except OSError as e:
            self.get_logger().warn(f"UDP send failed: {e}")

    def _on_cmd_vel(self, msg: Twist):
        left, right = vel_to_pwm(msg.linear.x, msg.angular.z)
        if left == 0 and right == 0:
            self._send("STOP\n")
        else:
            self._send(f"LEFT:{left} RIGHT:{right}\n")

    def _on_bot_command(self, msg: String):
        self._send(msg.data + "\n")

    # ------------------------------------------------------------------ receive
    def _rx_loop(self):
        while rclpy.ok():
            try:
                data, _ = self._sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break

            line = data.decode(errors="replace").strip()
            if line.startswith("TOF:"):
                self._publish_tof(line)
            elif line.startswith("ODOM:"):
                self._publish_odom(line)
            elif line.startswith("STATUS:"):
                self._status_pub.publish(String(data=line))
            else:
                self.get_logger().debug(f"unknown packet: {line!r}")

    def _publish_tof(self, line: str):
        # TOF:mm
        try:
            mm = float(line.split(":")[1])
        except (IndexError, ValueError):
            return
        msg = Range()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "tof_link"
        msg.radiation_type  = Range.INFRARED
        msg.field_of_view   = 0.44      # VL53L0X ~25° cone
        msg.min_range       = 0.03
        msg.max_range       = 2.0
        msg.range           = mm / 1000.0
        self._tof_pub.publish(msg)

    def _publish_odom(self, line: str):
        # ODOM:delta_left_m:delta_right_m:heading_rad
        try:
            parts   = line.split(":")
            dl      = float(parts[1])
            dr      = float(parts[2])
            heading = float(parts[3])
        except (IndexError, ValueError):
            return

        d   = (dl + dr) / 2.0
        dth = (dr - dl) / WHEEL_SEPARATION
        self._x   += d * math.cos(self._yaw + dth / 2.0)
        self._y   += d * math.sin(self._yaw + dth / 2.0)
        self._yaw  = heading        # firmware sends absolute heading from encoder integration

        msg = Odometry()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id  = "base_link"
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        cy = math.cos(self._yaw / 2.0)
        sy = math.sin(self._yaw / 2.0)
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        self._odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UdpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
