"""
encoder_plot.py — live plot of encoder tick counts and velocity from the bot.

Listens on UDP port 4211 for ODOM:<left>:<right>:0 packets.
Top subplot : cumulative left (blue) and right (red) tick counts.
Bottom subplot : instantaneous velocity in ticks/s for each wheel.

No ROS2 required.
"""

import socket
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

MAX_POINTS = 300

_times     = deque(maxlen=MAX_POINTS)
_l_ticks   = deque(maxlen=MAX_POINTS)
_r_ticks   = deque(maxlen=MAX_POINTS)
_l_vel     = deque(maxlen=MAX_POINTS)
_r_vel     = deque(maxlen=MAX_POINTS)
_lock      = threading.Lock()

_prev_l = 0
_prev_r = 0
_prev_t = time.time()
_start  = time.time()


def _udp_thread():
    global _prev_l, _prev_r, _prev_t

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", 4211))
    sock.settimeout(0.1)

    while True:
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            continue

        line = data.decode(errors="replace").strip()
        if not line.startswith("ODOM:"):
            continue

        parts = line.split(":")
        if len(parts) < 3:
            continue

        try:
            l, r = int(parts[1]), int(parts[2])
        except ValueError:
            continue

        now = time.time()
        dt  = now - _prev_t
        vl  = (l - _prev_l) / dt if dt > 0 else 0.0
        vr  = (r - _prev_r) / dt if dt > 0 else 0.0

        with _lock:
            _times.append(now - _start)
            _l_ticks.append(l)
            _r_ticks.append(r)
            _l_vel.append(vl)
            _r_vel.append(vr)

        _prev_l, _prev_r, _prev_t = l, r, now


threading.Thread(target=_udp_thread, daemon=True).start()

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=False)
fig.suptitle("Encoder Monitor  (ODOM packets on UDP 4211)")


def _update(_frame):
    with _lock:
        has_data = bool(_times)
        if has_data:
            t  = list(_times)
            lt = list(_l_ticks)
            rt = list(_r_ticks)
            lv = list(_l_vel)
            rv = list(_r_vel)

    ax1.clear()
    ax2.clear()
    if has_data:
        ax1.plot(t, lt, "b-", linewidth=1, label="Left ticks")
        ax1.plot(t, rt, "r-", linewidth=1, label="Right ticks")
        ax2.plot(t, lv, "b-", linewidth=1, label="Left  (ticks/s)")
        ax2.plot(t, rv, "r-", linewidth=1, label="Right (ticks/s)")
    ax1.set_ylabel("Ticks (cumulative)")
    ax1.legend(loc="upper left", fontsize=8)
    ax1.grid(True, alpha=0.4)
    ax2.axhline(0, color="k", linewidth=0.5)
    ax2.set_ylabel("Velocity (ticks/s)")
    ax2.set_xlabel("Time (s)")
    ax2.legend(loc="upper left", fontsize=8)
    ax2.grid(True, alpha=0.4)
    fig.tight_layout(rect=[0, 0, 1, 0.95])


ani = animation.FuncAnimation(fig, _update, interval=100, cache_frame_data=False)
plt.show()
