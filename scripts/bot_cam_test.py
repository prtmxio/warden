"""
bot_cam_test.py — low-latency S3 MJPEG stream with ArUco overlay.

Uses a direct HTTP byte-stream reader (no FFmpeg buffering) + background
capture thread so the display loop always gets the freshest frame.

ArUco detections persist on screen for ~0.75 s after the marker leaves
frame — green when fresh, orange when stale.

Usage:
    uv run scripts/bot_cam_test.py --ip 192.168.43.XX
"""

import argparse
import sys
import threading
import time
import urllib.request

import cv2
import matplotlib.pyplot as plt
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--ip", default="esp-cam.local",
                help="S3 IP or hostname (default: esp-cam.local)")
args = ap.parse_args()

STREAM_URL = f"http://{args.ip}:81/stream"

# ── ArUco setup ─────────────────────────────────────────────────────────────

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
params.adaptiveThreshWinSizeMin       = 3
params.adaptiveThreshWinSizeMax       = 53
params.adaptiveThreshWinSizeStep      = 10
params.polygonalApproxAccuracyRate    = 0.08
params.minMarkerPerimeterRate         = 0.02
params.cornerRefinementMethod         = cv2.aruco.CORNER_REFINE_SUBPIX
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

BOX_IDS      = {10, 11, 12, 13}
PERSIST_FRAMES = 15   # frames a detection stays shown after marker leaves (~0.75 s at 20 fps)
clahe        = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

# ── Low-latency MJPEG reader ─────────────────────────────────────────────────

def _mjpeg_frames(url, timeout=5):
    """Yield decoded BGR frames by parsing JPEG boundaries from raw HTTP stream."""
    r = urllib.request.urlopen(url, timeout=timeout)
    buf = b""
    while True:
        buf += r.read(4096)
        a = buf.find(b'\xff\xd8')    # JPEG SOI
        b = buf.find(b'\xff\xd9')    # JPEG EOI
        if a != -1 and b != -1 and b > a:
            jpg = buf[a:b + 2]
            buf = buf[b + 2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                yield frame

# ── Background capture thread ─────────────────────────────────────────────────

_latest_frame = [None]
_frame_lock   = threading.Lock()
_stop_event   = threading.Event()

def _capture_thread(url):
    while not _stop_event.is_set():
        try:
            for frame in _mjpeg_frames(url):
                if _stop_event.is_set():
                    break
                with _frame_lock:
                    _latest_frame[0] = frame
        except Exception as e:
            if not _stop_event.is_set():
                print(f"Stream error — reconnecting: {e}")
                time.sleep(0.5)

print(f"Connecting to {STREAM_URL} ...")
threading.Thread(target=_capture_thread, args=(STREAM_URL,), daemon=True).start()

# Wait for first frame
deadline = time.time() + 10
while _latest_frame[0] is None:
    if time.time() > deadline:
        print("Timeout: no frame received from S3.")
        _stop_event.set()
        sys.exit(1)
    time.sleep(0.05)

print("Stream open.")
print("Hold a box ArUco marker (IDs 10-13) in front of the camera.")
print("Close the window to quit.\n")

# ── Display loop ─────────────────────────────────────────────────────────────

fig, ax = plt.subplots(figsize=(7, 5))
fig.suptitle("S3 Bot Cam — ArUco Test")
ax.axis("off")
im_handle = None
plt.ion()

fps_t      = time.time()
fps_count  = 0
fps_display = 0.0
frame_count = 0

# Persistence state: id → (last_seen_frame, last_corners)
last_seen    = {}
last_corners = {}

try:
    while plt.get_fignums():
        with _frame_lock:
            frame = _latest_frame[0]
        if frame is None:
            plt.pause(0.01)
            continue

        frame = frame.copy()
        frame_count += 1

        # Detection on grayscale + CLAHE
        gray = clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        corners, ids, _ = detector.detectMarkers(gray)

        # Update persistence cache on fresh detections
        if ids is not None:
            for i, id_val in enumerate(ids.flatten()):
                if id_val in BOX_IDS:
                    if last_seen.get(id_val) != frame_count:  # first detection print
                        print(f"  Box ArUco ID: {id_val}")
                last_seen[id_val]    = frame_count
                last_corners[id_val] = corners[i]

        # Draw all IDs within persistence window
        for id_val, seen_at in list(last_seen.items()):
            age = frame_count - seen_at
            if age > PERSIST_FRAMES:
                continue
            c = last_corners[id_val]
            stale = age > 0
            color = (0, 165, 255) if stale else (0, 255, 0)  # orange=stale, green=fresh
            cv2.aruco.drawDetectedMarkers(frame, [c], np.array([[id_val]]))
            cx = int(c[0][:, 0].mean())
            cy = int(c[0][:, 1].mean()) - 10
            label = f"ID {id_val}" + (" (stale)" if stale else "")
            cv2.putText(frame, label, (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # FPS counter
        fps_count += 1
        now = time.time()
        if now - fps_t >= 1.0:
            fps_display = fps_count / (now - fps_t)
            fps_count   = 0
            fps_t       = now
        cv2.putText(frame, f"FPS: {fps_display:.1f}", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if im_handle is None:
            im_handle = ax.imshow(rgb)
        else:
            im_handle.set_data(rgb)

        fig.canvas.flush_events()
        plt.pause(0.001)

finally:
    _stop_event.set()
    plt.close()
