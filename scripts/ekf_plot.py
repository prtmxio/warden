"""
ekf_plot.py — live X-Y trajectory comparing three localization sources.

  Blue  dots  : /robot_pose   overhead ArUco camera  — absolute global position
  Orange line : /odom          encoder dead-reckoning — smooth but drifts over time
  Green  line : /odom_ekf      EKF fusion             — smooth + globally corrected

Requires the full SCADA stack running (ros2 launch scada/nav2_config/bringup.launch.py)
and ros2 available in PATH (source /opt/ros/humble/setup.bash first).
"""

import subprocess
import sys
import threading
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.animation as animation

sys.path.insert(0, str(Path(__file__).parent.parent / "scada"))
from arena_config import ARENA_SIZE_CM, BOX_POSITIONS

ARENA_M    = ARENA_SIZE_CM / 100.0
MAX_POINTS = 600

_lock   = threading.Lock()
_cam_x  = deque(maxlen=MAX_POINTS)
_cam_y  = deque(maxlen=MAX_POINTS)
_odom_x = deque(maxlen=MAX_POINTS)
_odom_y = deque(maxlen=MAX_POINTS)
_ekf_x  = deque(maxlen=MAX_POINTS)
_ekf_y  = deque(maxlen=MAX_POINTS)


def _parse_position(block_lines):
    """Return (x, y) from the first 'position:' block in a YAML echo block.

    Works for both nav_msgs/Odometry and PoseWithCovarianceStamped — both use
    the same pose.pose.position.x/y nesting in ros2 topic echo output.
    """
    for i, line in enumerate(block_lines):
        if line.strip() == "position:":
            x = y = None
            for sub in block_lines[i + 1 : i + 6]:
                s = sub.strip()
                if s.startswith("x:"):
                    try:
                        x = float(s[2:])
                    except ValueError:
                        pass
                elif s.startswith("y:"):
                    try:
                        y = float(s[2:])
                    except ValueError:
                        pass
            if x is not None and y is not None:
                return x, y
    return None


def _topic_thread(topic, x_buf, y_buf):
    try:
        proc = subprocess.Popen(
            ["ros2", "topic", "echo", topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
    except FileNotFoundError:
        print(f"[ekf_plot] 'ros2' not in PATH — {topic} will not be plotted", file=sys.stderr)
        return

    block = []
    for raw in proc.stdout:
        line = raw.rstrip()
        if line == "---":
            xy = _parse_position(block)
            if xy:
                with _lock:
                    x_buf.append(xy[0])
                    y_buf.append(xy[1])
            block = []
        else:
            block.append(line)


threading.Thread(target=_topic_thread, args=("/robot_pose", _cam_x,  _cam_y),  daemon=True).start()
threading.Thread(target=_topic_thread, args=("/odom",       _odom_x, _odom_y), daemon=True).start()
threading.Thread(target=_topic_thread, args=("/odom_ekf",   _ekf_x,  _ekf_y),  daemon=True).start()

fig, ax = plt.subplots(figsize=(7, 7))
fig.suptitle("EKF Plot  (blue=camera  orange=odom  green=EKF)")


def _update(_frame):
    ax.clear()
    with _lock:
        cx = list(_cam_x);  cy = list(_cam_y)
        ox = list(_odom_x); oy = list(_odom_y)
        ex = list(_ekf_x);  ey = list(_ekf_y)

    if ox:
        ax.plot(ox, oy, color="orange", linewidth=1.2, label="odom (dead-reckoning)")
    if ex:
        ax.plot(ex, ey, color="green",  linewidth=1.5, label="EKF /odom_ekf")
    if cx:
        ax.scatter(cx, cy, color="blue", s=8, label="camera /robot_pose")

    # highlight latest position for each source
    if ox: ax.scatter(ox[-1], oy[-1], color="orange", s=60, zorder=5)
    if ex: ax.scatter(ex[-1], ey[-1], color="green",  s=60, zorder=5)
    if cx: ax.scatter(cx[-1], cy[-1], color="blue",   s=60, zorder=5)

    ax.set_xlim(-0.1, ARENA_M + 0.1)
    ax.set_ylim(-0.1, ARENA_M + 0.1)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)
    ax.add_patch(plt.Rectangle(
        (0, 0), ARENA_M, ARENA_M,
        fill=False, edgecolor="gray", linewidth=1.5, linestyle="--",
    ))
    # Draw fixed box positions from arena_config
    for aruco_id, (bx, by) in BOX_POSITIONS.items():
        ax.scatter(bx, by, color="red", s=120, marker="s", zorder=4)
        ax.annotate(f"Box {aruco_id}", (bx, by), textcoords="offset points",
                    xytext=(5, 5), fontsize=7, color="red")


ani = animation.FuncAnimation(fig, _update, interval=200, cache_frame_data=False)
plt.tight_layout()
plt.show()
