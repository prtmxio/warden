"""
Fake ESP32-C6 UDP server for SCADA integration testing without hardware.

Sends TOF and ODOM packets continuously, responds to commands.
"""

import socket
import threading
import time
import sys

LISTEN_PORT  = 4210
SCADA_PORT   = 4211

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", LISTEN_PORT))
sock.settimeout(1.0)

scada_addr = ("127.0.0.1", SCADA_PORT)

print(f"mock_bot: listening on UDP :{LISTEN_PORT}, sending to {scada_addr}")


def send_loop():
    tof_interval  = 0.100   # 10 Hz
    odom_interval = 0.050   # 20 Hz
    last_tof  = time.monotonic()
    last_odom = time.monotonic()
    while True:
        now = time.monotonic()
        if now - last_tof >= tof_interval:
            sock.sendto(b"TOF:80\n", scada_addr)
            last_tof = now
        if now - last_odom >= odom_interval:
            sock.sendto(b"ODOM:0.01:0.00:0.005\n", scada_addr)
            last_odom = now
        time.sleep(0.005)


def repair_sequence():
    """Simulate bot doing repair: wait 2 s, send REPAIRED."""
    time.sleep(2.0)
    sock.sendto(b"STATUS:REPAIRED\n", scada_addr)
    print("mock_bot: STATUS:REPAIRED sent")


threading.Thread(target=send_loop, daemon=True).start()

print("mock_bot: sending TOF:150 @ 10 Hz and ODOM @ 20 Hz")
print("mock_bot: waiting for commands (SCAN / REPAIR_MODE / STOP)…\n")

try:
    while True:
        try:
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            continue

        cmd = data.decode(errors="replace").strip()
        print(f"mock_bot ← [{addr[0]}:{addr[1]}]  {cmd!r}")

        if cmd == "SCAN":
            print("mock_bot: SCAN received — inspection handled by SCADA side")
        elif cmd == "REPAIR_MODE":
            print("mock_bot: REPAIR_MODE received — starting repair sequence (2 s)")
            threading.Thread(target=repair_sequence, daemon=True).start()
        elif cmd == "STOP":
            print("mock_bot: STOP received — motors halted")
        else:
            print(f"mock_bot: unknown command {cmd!r}")

except KeyboardInterrupt:
    print("\nmock_bot: exit")
    sys.exit(0)
