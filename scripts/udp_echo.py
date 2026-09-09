"""
udp_echo.py — pretty-print every UDP packet arriving on port 4211.

Run this any time you want to see raw bot traffic (ODOM, TOF, STATUS).
No ROS2 required.
"""

import socket
import datetime

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("", 4211))

print("Listening on UDP port 4211 — Ctrl+C to stop\n")

while True:
    data, addr = sock.recvfrom(1024)
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{ts}]  {addr[0]}:{addr[1]}  {data.decode(errors='replace').strip()}")
