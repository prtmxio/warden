"""
Entry point — launches all SCADA nodes + the Textual TUI.

Usage:
  source /opt/ros/humble/setup.bash
  uv run scada/tui_node.py [--sim]

Pass --tui to scada/main.py to reach this file from the existing launcher.
Each backend node gets its own SingleThreadedExecutor spinning in a daemon
thread.  The TuiRosNode (UI node) also gets its own executor + daemon thread.
WardenApp.run() owns the main thread; on exit all executors are shut down and
rclpy is cleaned up.
"""

import os
import sys
import threading

# cv2 bundles its own Qt .so files; loading PyQt5's C extensions first wins
# the dlopen cache race if PyQt5 happens to be imported elsewhere.  Force the
# plugin path env var here as a backstop (mirrors main.py lines 19-26).
try:
    import PyQt5 as _pyqt5
    os.environ.setdefault(
        "QT_QPA_PLATFORM_PLUGIN_PATH",
        os.path.join(os.path.dirname(_pyqt5.__file__), "Qt5", "plugins"),
    )
    del _pyqt5
except ImportError:
    pass

import rclpy

sys.path.insert(0, "/home/xio/Desktop/warden/scada")

from bot_cam         import BotCamNode
from control_node    import ControlNode
from udp_bridge_node import UdpBridgeNode
from vision_node     import VisionNode

from tui.app        import WardenApp
from tui.ros_bridge import TuiRosNode, HAS_ROS


def _spin(node):
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except Exception as exc:
        node.get_logger().error(f"{node.get_name()} crashed: {exc}")
    finally:
        executor.shutdown()


def main() -> None:
    sim = "--sim" in sys.argv

    rclpy.init()

    bg_nodes = [
        ControlNode(),
        UdpBridgeNode(),
        VisionNode(),
        BotCamNode(),
    ]

    threads = [
        threading.Thread(target=_spin, args=(n,), daemon=True) for n in bg_nodes
    ]
    for t in threads:
        t.start()

    app = WardenApp()

    if HAS_ROS:
        ros_node = TuiRosNode(app, sim=sim)
        ros_executor = rclpy.executors.SingleThreadedExecutor()
        ros_executor.add_node(ros_node)
        ros_thread = threading.Thread(target=ros_executor.spin, daemon=True)
        ros_thread.start()
        app.ros_node = ros_node
    else:
        ros_executor = None
        ros_node = None

    app.run()

    if ros_executor is not None:
        ros_executor.shutdown()

    for n in bg_nodes:
        n.destroy_node()
    if ros_node is not None and HAS_ROS:
        ros_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
