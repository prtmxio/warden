"""
tof_plot.py — live plot of VL53L0X range from the bot.

Listens on UDP port 4211 for TOF:<mm> packets.
2000 mm = no-target sentinel (firmware caps raw 8190 → 2000; shown as dashed red line).

Optional: pass --bot-ip <ip> to auto-send discovery pings so the bot
learns the laptop's IP without a manual nc command.

No ROS2 required.
"""

import argparse
import socket
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

ap = argparse.ArgumentParser(description="Live VL53L0X range plot")
ap.add_argument("--bot-ip", default=None,
                help="Bot IP for auto-discovery, e.g. 172.17.142.91")
args = ap.parse_args()

BOT_UDP_PORT = 4210

MAX_POINTS = 300

_times  = deque(maxlen=MAX_POINTS)
_ranges = deque(maxlen=MAX_POINTS)
_lock   = threading.Lock()
_start  = time.time()


def _udp_thread():
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
        if not line.startswith("TOF:"):
            continue

        try:
            mm = int(line.split(":")[1])
        except (IndexError, ValueError):
            continue

        with _lock:
            _times.append(time.time() - _start)
            _ranges.append(mm)


threading.Thread(target=_udp_thread, daemon=True).start()


def _discovery_thread(bot_ip):
    """Sends PING every 2 s until the first TOF packet arrives."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while True:
        with _lock:
            if _times:
                break
        try:
            sock.sendto(b"PING", (bot_ip, BOT_UDP_PORT))
        except Exception:
            pass
        time.sleep(2)
    sock.close()


if args.bot_ip:
    threading.Thread(target=_discovery_thread, args=(args.bot_ip,), daemon=True).start()

fig, ax = plt.subplots(figsize=(10, 4))
fig.suptitle("VL53L0X TOF Monitor  (TOF packets on UDP 4211)")


def _update(_frame):
    with _lock:
        has_data = bool(_times)
        if has_data:
            t = list(_times)
            r = list(_ranges)

    ax.clear()
    if has_data:
        ax.plot(t, r, "g-", linewidth=1.5, label="Range (mm)")
    ax.axhline(2000, color="r", linestyle="--", linewidth=0.8, label="No target / out of range (2000 mm)")
    ax.set_ylabel("Range (mm)")
    ax.set_xlabel("Time (s)")
    ax.set_ylim(0, 2200)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.4)
    if has_data:
        latest = r[-1]
        color  = "red" if latest >= 1900 else "green"
        ax.text(0.02, 0.92, f"{latest} mm", transform=ax.transAxes,
                fontsize=22, fontweight="bold", color=color,
                verticalalignment="top")
    fig.tight_layout(rect=[0, 0, 1, 0.95])


ani = animation.FuncAnimation(fig, _update, interval=100, cache_frame_data=False)
plt.show()
