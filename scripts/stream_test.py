import cv2
import numpy as np
import time
import argparse
import sys

DEFAULT_URL   = ""
DISPLAY_SCALE = 0.5       # shrinks display only — detection runs on full-res frame
DICT_ID       = cv2.aruco.DICT_4X4_50  # must match aruco_gen.py and vision_node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default=DEFAULT_URL)
    args = parser.parse_args()

    if not hasattr(cv2, "aruco"):
        print("ERROR: cv2.aruco not found — install opencv-contrib-python, not opencv-python")
        sys.exit(1)

    aruco_dict   = cv2.aruco.getPredefinedDictionary(DICT_ID)
    aruco_params = cv2.aruco.DetectorParameters()
    detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    if not args.url:
        print("ERROR: pass stream URL with --url http://<ip>:<port>/video")
        sys.exit(1)

    cap = cv2.VideoCapture(args.url)
    if not cap.isOpened():
        print(f"ERROR: cannot open {args.url}")
        sys.exit(1)

    fps_counter = 0
    fps_display = 0.0
    t0 = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            # Stream drop (phone locked, Wi-Fi hiccup) — reopen instead of crash
            cap.release()
            time.sleep(0.5)
            cap = cv2.VideoCapture(args.url)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # ~30% faster detection

        # corners: list of (1,4,2) arrays — pixel coords of each marker's 4 corners
        # ids: (N,1) int array
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i, mid in enumerate(ids.flatten()):
                cx = int(corners[i][0][:, 0].mean())
                cy = int(corners[i][0][:, 1].mean())
                cv2.putText(frame, f"ID {mid}", (cx - 20, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        fps_counter += 1
        elapsed = time.time() - t0
        if elapsed >= 1.0:
            fps_display = fps_counter / elapsed
            fps_counter = 0
            t0 = time.time()

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
