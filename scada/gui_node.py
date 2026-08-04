"""
gui_node — PyQt5 dashboard (4-panel layout).

Day 3 skeleton: opens a window with placeholder panels and spins.
Full UI implementation goes in on Day 4.
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
    from PyQt5.QtCore import Qt
    HAS_QT = True
except ImportError:
    HAS_QT = False


class GuiNode(Node):
    def __init__(self):
        super().__init__("gui_node")

        self._decision_pub = self.create_publisher(String, "/operator_decision", 10)

        self.create_subscription(
            String, "/inspection_result", self._on_inspection_result, 10
        )

        self.get_logger().info("gui_node skeleton running (full UI on Day 4)")

    def _on_inspection_result(self, msg: String):
        self.get_logger().info(f"inspection result: {msg.data}")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Warden SCADA")
        self.resize(1280, 720)
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        label = QLabel(
            "Day 3 skeleton — full 4-panel dashboard coming Day 4",
            alignment=Qt.AlignCenter,
        )
        layout.addWidget(label)


def _ros_spin(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()

    if HAS_QT:
        app = QApplication(sys.argv)
        win = MainWindow()
        win.show()
        ros_thread = threading.Thread(target=_ros_spin, args=(node,), daemon=True)
        ros_thread.start()
        app.exec_()
    else:
        node.get_logger().warn("PyQt5 not found — running headless")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
