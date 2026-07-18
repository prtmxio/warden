import cv2
import numpy as np
import time
import argparse
import sys

DEFAULT_URL   = ""
DICT_ID       = cv2.aruco.DICT_4X4_50  # must match aruco_gen.py and vision_node
WARP_SIZE     = 800
ARENA_SIZE_CM = 150
PX_PER_CM     = WARP_SIZE / ARENA_SIZE_CM
BOT_ARUCO_ID  = 5
CORNER_IDS    = [0, 1, 2, 3]

# Maps each corner ArUco ID to its fixed pixel position in the warped output.
# Change this if you physically place the markers in a different order.
#   ID 0 → top-left   ID 1 → top-right
#   ID 3 → bot-left   ID 2 → bot-right
CORNER_DST = {
    0: [0,         0        ],
    1: [WARP_SIZE, 0        ],
    2: [WARP_SIZE, WARP_SIZE],
    3: [0,         WARP_SIZE],
}


def marker_center(corners_entry):
    pts = corners_entry[0]  # (4, 2)
    return pts[:, 0].mean(), pts[:, 1].mean()


def marker_heading(corners_entry):
    # ArUco returns corners in order: TL, TR, BR, BL
    # Top-edge midpoint minus bottom-edge midpoint gives the forward direction vector
    pts = corners_entry[0]
    top_mid    = (pts[0] + pts[1]) / 2.0
    bottom_mid = (pts[3] + pts[2]) / 2.0
    dx = top_mid[0] - bottom_mid[0]
    dy = top_mid[1] - bottom_mid[1]
    return np.degrees(np.arctan2(-dy, dx))  # -dy because image Y increases downward


def compute_homography(corners, ids):
    # Collects the 4 corner markers and computes the 3x3 matrix H that maps
    # their raw pixel positions to the fixed CORNER_DST quad (perspective correction).
    id_to_corner = {}
    for i, mid in enumerate(ids.flatten()):
        if mid in CORNER_IDS:
            cx, cy = marker_center(corners[i])
            id_to_corner[mid] = [cx, cy]
    if len(id_to_corner) < 4:
        return None, None
    src_pts = np.array([id_to_corner[cid] for cid in CORNER_IDS], dtype=np.float32)
    dst_pts = np.array([CORNER_DST[cid]   for cid in CORNER_IDS], dtype=np.float32)
    H, _ = cv2.findHomography(src_pts, dst_pts)
    return H, src_pts


def warp_point(H, x, y):
    pt = np.array([[[x, y]]], dtype=np.float32)
    return cv2.perspectiveTransform(pt, H)[0][0]


def draw_bot_arrow(img, cx, cy, theta_deg, color=(0, 255, 255), length=30):
    cx, cy = int(cx), int(cy)
    cv2.circle(img, (cx, cy), 8, color, -1)
    rad = np.radians(theta_deg)
    ex = int(cx + length * np.cos(rad))
    ey = int(cy - length * np.sin(rad))
    cv2.arrowedLine(img, (cx, cy), (ex, ey), color, 2, tipLength=0.4)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default=DEFAULT_URL)
    args = parser.parse_args()

    if not hasattr(cv2, "aruco"):
        print("ERROR: cv2.aruco missing — install opencv-contrib-python")
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

    H = None  # recomputed every frame when all 4 corner markers are visible
    fps_counter = 0
    fps_display = 0.0
    t0 = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            cap.release()
            time.sleep(0.5)
            cap = cv2.VideoCapture(args.url)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        raw_display = frame.copy()
        warped = np.zeros((WARP_SIZE, WARP_SIZE, 3), dtype=np.uint8)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(raw_display, corners, ids)

            new_H, src_pts = compute_homography(corners, ids)
            if new_H is not None:
                H = new_H
                for pt in src_pts:
                    cv2.circle(raw_display, (int(pt[0]), int(pt[1])), 12, (0, 128, 255), 3)

            if H is not None:
                warped = cv2.warpPerspective(frame, H, (WARP_SIZE, WARP_SIZE))

                step = int(30 * PX_PER_CM)  # grid line every 30cm
                for i in range(0, WARP_SIZE, step):
                    cv2.line(warped, (i, 0), (i, WARP_SIZE), (40, 40, 40), 1)
                    cv2.line(warped, (0, i), (WARP_SIZE, i), (40, 40, 40), 1)

                for i, mid in enumerate(ids.flatten()):
                    if mid == BOT_ARUCO_ID:
                        cx_raw, cy_raw = marker_center(corners[i])
                        theta = marker_heading(corners[i])
                        wx, wy = warp_point(H, cx_raw, cy_raw)
                        x_cm = wx / PX_PER_CM
                        y_cm = wy / PX_PER_CM
                        draw_bot_arrow(warped, wx, wy, theta)
                        cv2.putText(warped, f"x={x_cm:.1f}  y={y_cm:.1f}  θ={theta:.1f}°",
                                    (10, WARP_SIZE - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
                        print(f"BOT  x={x_cm:.1f}cm  y={y_cm:.1f}cm  theta={theta:.1f}°  ({fps_display:.1f} Hz)")
                        break

        corners_visible = sum(1 for mid in (ids.flatten() if ids is not None else [])
                              if mid in CORNER_IDS)
        status = f"corners: {corners_visible}/4" + ("  [WARPED]" if H is not None else "  [NO WARP]")
        cv2.putText(warped, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 0), 1)

        fps_counter += 1
        elapsed = time.time() - t0
        if elapsed >= 1.0:
            fps_display = fps_counter / elapsed
            fps_counter = 0
            t0 = time.time()
        cv2.putText(raw_display, f"{fps_display:.1f} fps", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

        cv2.imshow("raw", raw_display)
        cv2.imshow("warped", warped)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
