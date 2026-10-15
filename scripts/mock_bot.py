"""
Fake ESP32-C6 UDP server for SCADA integration testing without hardware.

Simulates a differential-drive robot: integrates received LEFT/RIGHT wheel
commands into a pose, sends accurate ODOM and synthesised TOF back to SCADA.

Starting pose: stationary at origin (0, 0) facing north (yaw=π/2).  The
alignment state in control_node will turn and drive the sim bot to centre on
box 10 before approaching — just like real hardware would.
"""

import math
import socket
import threading
import time
import sys

LISTEN_PORT  = 4210
SCADA_PORT   = 4211

# Must match arena_config.py
WHEEL_SEP   = 0.12
MAX_PWM     = 180
MAX_VEL     = 0.15          # m/s at full PWM
PWM_SCALE   = MAX_VEL / MAX_PWM

# Box wall y-coordinate (front face of box 10 is at y≈1.65 m)
BOX_WALL_Y  = 1.65

# Simulated pose — starts at origin facing north
_sim_x   = 0.0
_sim_y   = 0.0
_sim_yaw = math.pi / 2.0   # facing +Y (north = TARGET_YAW in control_node)
_left_pwm  = 0
_right_pwm = 0
_cmd_lock  = threading.Lock()  # guards _left_pwm, _right_pwm only

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", LISTEN_PORT))
sock.settimeout(1.0)

scada_addr = ("127.0.0.1", SCADA_PORT)


def send_loop():
    global _sim_x, _sim_y, _sim_yaw

    tof_interval  = 0.100   # 10 Hz
    odom_interval = 0.050   # 20 Hz
    last_tof  = time.monotonic()
    last_odom = time.monotonic()
    prev_t    = time.monotonic()

    # Accumulate wheel displacement between odom sends so udp_bridge gets
    # the full 50 ms worth of movement, not just the last 5 ms slice.
    acc_dl = 0.0
    acc_dr = 0.0

    while True:
        now = time.monotonic()
        dt  = now - prev_t
        prev_t = now

        with _cmd_lock:
            lv = _left_pwm  * PWM_SCALE
            rv = _right_pwm * PWM_SCALE

        # Integrate forward kinematics
        v   = (lv + rv) / 2.0
        om  = (rv - lv) / WHEEL_SEP
        _sim_x   += v * math.cos(_sim_yaw) * dt
        _sim_y   += v * math.sin(_sim_yaw) * dt
        _sim_yaw += om * dt
        _sim_yaw  = math.atan2(math.sin(_sim_yaw), math.cos(_sim_yaw))

        acc_dl += lv * dt
        acc_dr += rv * dt

        # TOF: distance along +Y from bot to box wall
        fwd_y = math.sin(_sim_yaw)
        if fwd_y > 0.01:
            tof_m = (BOX_WALL_Y - _sim_y) / fwd_y
            tof_val = max(20.0, tof_m * 1000.0)
        else:
            tof_val = 1500.0

        if now - last_tof >= tof_interval:
            sock.sendto(f"TOF:{tof_val:.0f}\n".encode(), scada_addr)
            last_tof = now

        if now - last_odom >= odom_interval:
            sock.sendto(
                f"ODOM:{acc_dl:.5f}:{acc_dr:.5f}:{_sim_yaw:.5f}\n".encode(),
                scada_addr,
            )
            acc_dl = 0.0
            acc_dr = 0.0
            last_odom = now

        time.sleep(0.005)


def repair_sequence():
    time.sleep(2.0)
    sock.sendto(b"STATUS:REPAIRED\n", scada_addr)
    print("mock_bot: STATUS:REPAIRED sent")


threading.Thread(target=send_loop, daemon=True).start()

print(f"mock_bot: listening on UDP :{LISTEN_PORT}, sending to {scada_addr}")
print(f"mock_bot: sim start  x={_sim_x:.3f} y={_sim_y:.3f} yaw={math.degrees(_sim_yaw):.1f}°")
print("mock_bot: waiting for commands…\n")

try:
    while True:
        try:
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            continue

        cmd = data.decode(errors="replace").strip()

        if cmd.startswith("LEFT:") and "RIGHT:" in cmd:
            try:
                parts = cmd.split()
                l = int(parts[0].split(":")[1])
                r = int(parts[1].split(":")[1])
                with _cmd_lock:
                    _left_pwm  = l
                    _right_pwm = r
            except (IndexError, ValueError):
                pass
            # Motor commands are frequent — don't print them
        elif cmd == "STOP":
            with _cmd_lock:
                _left_pwm  = 0
                _right_pwm = 0
            print("mock_bot: STOP")
        elif cmd == "SCAN":
            print("mock_bot: SCAN received")
        elif cmd == "REPAIR_MODE":
            print("mock_bot: REPAIR_MODE — starting repair (2 s)")
            with _cmd_lock:
                _left_pwm  = 0
                _right_pwm = 0
            threading.Thread(target=repair_sequence, daemon=True).start()
        elif cmd in ("REPAIR_STOP",):
            with _cmd_lock:
                _left_pwm  = 0
                _right_pwm = 0
            print(f"mock_bot: {cmd}")
        else:
            print(f"mock_bot ← {cmd!r}")

except KeyboardInterrupt:
    print("\nmock_bot: exit")
    sys.exit(0)
