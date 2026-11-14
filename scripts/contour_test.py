import cv2
import numpy as np
import time
import argparse
import sys

DEFAULT_URL = ""
DICT_ID     = cv2.aruco.DICT_4X4_50  # must match aruco_gen.py and vision_node
WARP_SIZE   = 800
CORNER_IDS  = [0, 1, 2, 3]

CORNER_DST = {
    0: [0,         0        ],
    1: [WARP_SIZE, 0        ],
    2: [WARP_SIZE, WARP_SIZE],
    3: [0,         WARP_SIZE],
}

DEFAULTS = dict(H_lo=0, H_hi=179, S_lo=0, S_hi=255, V_lo=60, V_hi=255, min_area=800)


def marker_center(corners_entry):
    pts = corners_entry[0]
    return pts[:, 0].mean(), pts[:, 1].mean()


def compute_homography(corners, ids):
    id_to_corner = {}
    for i, mid in enumerate(ids.flatten()):
        if mid in CORNER_IDS:
            cx, cy = marker_center(corners[i])
            id_to_corner[mid] = [cx, cy]
    if len(id_to_corner) < 4:
        return None
    src_pts = np.array([id_to_corner[cid] for cid in CORNER_IDS], dtype=np.float32)
    dst_pts = np.array([CORNER_DST[cid]   for cid in CORNER_IDS], dtype=np.float32)
    H, _ = cv2.findHomography(src_pts, dst_pts)
    return H


def get_sliders(win):
    keys = ["H_lo", "H_hi", "S_lo", "S_hi", "V_lo", "V_hi", "min_area"]
    return {k: cv2.getTrackbarPos(k, win) for k in keys}


def detect_boxes(warped_bgr, s):
    # HSV separates color (H) from brightness (V), making the threshold robust
    # to lighting changes — same box color reads differently in RGB under dim light
    hsv  = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,
                       np.array([s["H_lo"], s["S_lo"], s["V_lo"]]),
                       np.array([s["H_hi"], s["S_hi"], s["V_hi"]]))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)  # kills noise specks
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)  # fills holes in blobs

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area >= s["min_area"]:
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                boxes.append((cnt, M["m10"] / M["m00"], M["m01"] / M["m00"], area))
    return boxes


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

    ctrl_win = "controls"
    cv2.namedWindow(ctrl_win)
    # OpenCV requires a window with an actual image to host trackbars
    ctrl_img = np.zeros((1, 400, 3), dtype=np.uint8)

    def noop(_): pass
    cv2.createTrackbar("H_lo",     ctrl_win, DEFAULTS["H_lo"],     179,  noop)
    cv2.createTrackbar("H_hi",     ctrl_win, DEFAULTS["H_hi"],     179,  noop)
    cv2.createTrackbar("S_lo",     ctrl_win, DEFAULTS["S_lo"],     255,  noop)
    cv2.createTrackbar("S_hi",     ctrl_win, DEFAULTS["S_hi"],     255,  noop)
    cv2.createTrackbar("V_lo",     ctrl_win, DEFAULTS["V_lo"],     255,  noop)
    cv2.createTrackbar("V_hi",     ctrl_win, DEFAULTS["V_hi"],     255,  noop)
    cv2.createTrackbar("min_area", ctrl_win, DEFAULTS["min_area"], 5000, noop)

    if not args.url:
        print("ERROR: pass stream URL with --url http://<ip>:<port>/video")
        sys.exit(1)

    cap = cv2.VideoCapture(args.url)
    if not cap.isOpened():
        print(f"ERROR: cannot open {args.url}")
        sys.exit(1)

    H = None
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
        warped = np.zeros((WARP_SIZE, WARP_SIZE, 3), dtype=np.uint8)

        if ids is not None:
            new_H = compute_homography(corners, ids)
            if new_H is not None:
                H = new_H

        if H is not None:
            warped = cv2.warpPerspective(frame, H, (WARP_SIZE, WARP_SIZE))
            s = get_sliders(ctrl_win)
            boxes = detect_boxes(warped, s)

            for cnt, cx, cy, area in boxes:
                # bounding rect — easier to read than raw contour outline
                rx, ry, rw, rh = cv2.boundingRect(cnt)
                cv2.rectangle(warped, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
                cv2.circle(warped, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                cv2.putText(warped, f"{int(area)}px²", (rx, ry - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

            count_color = (0, 255, 0) if len(boxes) == 4 else (0, 0, 255)
            cv2.putText(warped, f"boxes: {len(boxes)}/4", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, count_color, 2)
        else:
            cv2.putText(warped, "no warp — show corner markers (IDs 0-3)", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 200), 1)

        fps_counter += 1
        elapsed = time.time() - t0
        if elapsed >= 1.0:
            fps_display = fps_counter / elapsed
            fps_counter = 0
            t0 = time.time()
        cv2.putText(warped, f"{fps_display:.1f} fps", (10, WARP_SIZE - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 150), 1)

        cv2.imshow("warped", warped)
        cv2.imshow(ctrl_win, ctrl_img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    s = get_sliders(ctrl_win)
    print("\nFinal values — paste into arena_config.py:")
    print(f"BOX_HSV_LO   = ({s['H_lo']}, {s['S_lo']}, {s['V_lo']})")
    print(f"BOX_HSV_HI   = ({s['H_hi']}, {s['S_hi']}, {s['V_hi']})")
    print(f"BOX_MIN_AREA = {s['min_area']}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
