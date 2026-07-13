"""
Day 1 — Phone MJPEG stream test.
Opens IP Webcam feed, detects ArUco markers, draws overlays, shows FPS.

Usage:
  python scripts/stream_test.py
  python scripts/stream_test.py --url http://192.168.43.1:8080/video

Target: FPS >= 18. Green bounding box + ID on any detected ArUco.
"""

import cv2
import numpy as np
import time
import argparse
import sys

# ── constants ─────────────────────────────────────────────────────────────────
DEFAULT_URL   = "http://192.168.43.1:8080/video"
# IP Webcam default URL. Phone hotspot gateway is always 192.168.43.1.
# If /video doesn't work, try /shot.jpg (single frame) or /?action=stream

DISPLAY_SCALE = 0.5   # shrink display window — 0.5 = half size, 1.0 = full
DICT_ID       = cv2.aruco.DICT_4X4_50
# Must match aruco_gen.py exactly. 4x4 grid, 50-pattern dictionary.
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default=DEFAULT_URL)
    args = parser.parse_args()

    # Guard: opencv-contrib-python is required for cv2.aruco.
    # Plain opencv-python does NOT include it — common install mistake.
    if not hasattr(cv2, "aruco"):
        print("ERROR: cv2.aruco not found.")
        print("Fix:  pip uninstall opencv-python && pip install opencv-contrib-python")
        sys.exit(1)

    # Build the ArUco detector once — reusing it every frame is faster than
    # recreating it. Parameters use OpenCV defaults (works well for printed markers).
    aruco_dict   = cv2.aruco.getPredefinedDictionary(DICT_ID)
    aruco_params = cv2.aruco.DetectorParameters()
    detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    print(f"Opening: {args.url}")
    # VideoCapture handles MJPEG over HTTP automatically.
    # It decodes each JPEG frame from the multipart HTTP stream.
    cap = cv2.VideoCapture(args.url)
    if not cap.isOpened():
        print("ERROR: cannot open stream.")
        print("  1. Connect laptop to phone hotspot Wi-Fi")
        print("  2. Open IP Webcam on phone → tap 'Start server'")
        print(f"  3. Try opening {args.url} in a browser to confirm it's reachable")
        sys.exit(1)

    # FPS calculation: count frames, divide by elapsed seconds, reset every second.
    fps_counter = 0
    fps_display = 0.0
    t0 = time.time()

    print("Stream open. Press Q to quit.")

    while True:
        ret, frame = cap.read()
        # ret=False means the stream dropped (phone screen locked, Wi-Fi hiccup, etc.)
        # We reopen instead of crashing — keeps the script running during brief drops.
        if not ret:
            print("Frame drop — reconnecting...")
            cap.release()
            time.sleep(0.5)
            cap = cv2.VideoCapture(args.url)
            continue

        # Detection requires grayscale. The color frame is kept for drawing overlays.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # corners: list of (1,4,2) arrays — the 4 corner pixel coords of each marker
        # ids:     (N,1) array of detected marker IDs
        # _:       rejected candidates (we don't need them)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            # drawDetectedMarkers draws the green bounding quad + small axes
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Add ID label at the center of each detected marker
            for i, mid in enumerate(ids.flatten()):
                cx = int(corners[i][0][:, 0].mean())  # mean of 4 corner x-coords
                cy = int(corners[i][0][:, 1].mean())  # mean of 4 corner y-coords
                cv2.putText(frame, f"ID {mid}", (cx - 20, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Update FPS display once per second
        fps_counter += 1
        elapsed = time.time() - t0
        if elapsed >= 1.0:
            fps_display = fps_counter / elapsed
            fps_counter = 0
            t0 = time.time()

        # Overlay FPS + resolution in top-left corner
        h, w = frame.shape[:2]
        cv2.putText(frame, f"{fps_display:.1f} fps  {w}x{h}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

        if DISPLAY_SCALE != 1.0:
            dw = int(frame.shape[1] * DISPLAY_SCALE)
            dh = int(frame.shape[0] * DISPLAY_SCALE)
            frame = cv2.resize(frame, (dw, dh))

        cv2.imshow("stream_test  [Q = quit]", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
