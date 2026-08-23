"""
Entry point — launches all SCADA nodes in one process.

Usage:
  source /opt/ros/humble/setup.bash
  uv run scada/main.py

Each node gets its own SingleThreadedExecutor so they don't fight over
the default global executor (causes "generator already executing" crash).
GuiNode owns the Qt main thread; all other nodes spin in daemon threads.
"""

import os
import sys
import threading

# cv2 bundles its own Qt .so files; loading PyQt5's C extensions first wins
# the dlopen cache race.  Force the plugin path env var here as a backstop.
try:
    import PyQt5 as _pyqt5
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.path.join(
        os.path.dirname(_pyqt5.__file__), "Qt5", "plugins"
    )
    del _pyqt5
except ImportError:
    pass

import rclpy

sys.path.insert(0, "/home/xio/Desktop/warden/scada")

from control_node    import ControlNode
from udp_bridge_node import UdpBridgeNode
from vision_node     import VisionNode
from gui_node        import RosBridge, GuiNode, MainWindow, HAS_QT

if HAS_QT:
    from PyQt5.QtWidgets import QApplication


def spin(node):
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

    bg_nodes = [
        ControlNode(),
        UdpBridgeNode(),
        VisionNode(),
    ]

    threads = [
        threading.Thread(target=spin, args=(n,), daemon=True) for n in bg_nodes
    ]
    for t in threads:
        t.start()

    if HAS_QT:
        # QApplication must exist before any QObject (including RosBridge) is created.
        app    = QApplication(sys.argv)
        bridge = RosBridge()
        gui    = GuiNode(bridge)
        win    = MainWindow(bridge, gui)
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
        bridge = RosBridge()
        gui    = GuiNode(bridge)
        try:
            spin(gui)
        except KeyboardInterrupt:
            pass

    for n in bg_nodes + [gui]:
        n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
