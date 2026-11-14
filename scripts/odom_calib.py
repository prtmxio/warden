"""
odom_calib.py — accumulate ODOM deltas to calibrate TICKS_PER_M.

Pokes the bot so it starts streaming, then sums every ODOM:Δleft:Δright
delta it sends. Push the bot a measured distance by hand, then Ctrl+C to
print the totals and the corrected TICKS_PER_M.

No ROS2 required.
"""

import socket
import sys

BOT_HOST = "esp-bot.local"
BOT_CMD_PORT = 4210
LISTEN_PORT = 4211
TICKS_PER_M_CURRENT = 1600.0   # whatever is flashed in main.c right now

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", LISTEN_PORT))

# Poke the bot so it latches our address and starts streaming telemetry.
sock.sendto(b"STOP", (BOT_HOST, BOT_CMD_PORT))

print("Listening + summing ODOM. Push the bot, then Ctrl+C.\n")

sum_l = sum_r = 0.0
last_hdg = 0.0
try:
    while True:
        data, _ = sock.recvfrom(1024)
        msg = data.decode(errors="replace").strip()
        if not msg.startswith("ODOM:"):
            continue
        parts = msg.split(":")
        dl, dr, hdg = float(parts[1]), float(parts[2]), float(parts[3])
        sum_l += dl
        sum_r += dr
        last_hdg = hdg
        print(f"\rΣleft={sum_l:+.4f}m  Σright={sum_r:+.4f}m  heading={last_hdg:+.3f}rad",
              end="", flush=True)
except KeyboardInterrupt:
    reported = (sum_l + sum_r) / 2.0
    actual = float(sys.argv[1]) if len(sys.argv) > 1 else 0.60
    print(f"\n\nΣleft={sum_l:+.4f}m  Σright={sum_r:+.4f}m  heading={last_hdg:+.3f}rad")
    print(f"reported avg distance = {reported:.4f} m  (actual = {actual} m)")
    if reported != 0:
        print(f"TICKS_PER_M_new = {TICKS_PER_M_CURRENT * reported / actual:.1f}")
