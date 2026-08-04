"""
Entry point — launches all SCADA nodes in one process.

Usage:
  source /opt/ros/humble/setup.bash
  python scada/main.py

Each node gets its own thread so they all spin concurrently.
For production, prefer a proper launch file so nodes can be restarted independently.
"""

import sys
import threading

import rclpy

sys.path.insert(0, "/home/xio/Desktop/warden/scada")

from control_node    import ControlNode
from udp_bridge_node import UdpBridgeNode
from vision_node     import VisionNode
from gui_node        import GuiNode

try:
    from PyQt5.QtWidgets import QApplication
    HAS_QT = True
except ImportError:
    HAS_QT = False


def spin(node):
    # Each node gets its own SingleThreadedExecutor so they don't fight over
    # the default global executor (causes "generator already executing" crash).
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except Exception as exc:
        node.get_logger().error(f"{node.get_name()} crashed: {exc}")
    finally:
        executor.shutdown()


def main():
    rclpy.init()

    nodes = [
        ControlNode(),
        UdpBridgeNode(),
        VisionNode(),
    ]

    threads = [threading.Thread(target=spin, args=(n,), daemon=True) for n in nodes]
    for t in threads:
        t.start()

    # GuiNode owns the main thread (Qt requires it)
    gui = GuiNode()
    if HAS_QT:
        app = QApplication(sys.argv)
        from gui_node import MainWindow
        win = MainWindow()
        win.show()
        gui_executor = rclpy.executors.SingleThreadedExecutor()
        gui_executor.add_node(gui)
        ros_gui_thread = threading.Thread(
            target=gui_executor.spin, daemon=True
        )
        ros_gui_thread.start()
        app.exec_()
        gui_executor.shutdown()
    else:
        try:
            spin(gui)
        except KeyboardInterrupt:
            pass

    for n in nodes + [gui]:
        n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
