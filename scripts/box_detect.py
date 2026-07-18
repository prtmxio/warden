import cv2
import numpy as np
import time
import argparse
import sys
import os
import re
from itertools import permutations

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from scada.arena_config import (
    BOX_HSV_LO, BOX_HSV_HI, BOX_MIN_AREA,
    WARP_SIZE, ARENA_SIZE_CM, CORNER_ARUCO_IDS,
)

DICT_ID    = cv2.aruco.DICT_4X4_50
PX_PER_CM  = WARP_SIZE / ARENA_SIZE_CM
ARENA_CFG  = os.path.join(os.path.dirname(__file__), "..", "scada", "arena_config.py")

# Ideal quadrant centers in the 800×800 warped space (TL=10, TR=11, BR=12, BL=13).
# IDs are assigned by minimum total distance across all 4 boxes (24 permutations),
# so two boxes in the same geometric quadrant still get unique IDs.
BOX_IDS = [10, 11, 12, 13]
QUAD_CENTERS = {
    10: (WARP_SIZE * 0.25, WARP_SIZE * 0.25),
    11: (WARP_SIZE * 0.75, WARP_SIZE * 0.25),
    12: (WARP_SIZE * 0.75, WARP_SIZE * 0.75),
    13: (WARP_SIZE * 0.25, WARP_SIZE * 0.75),
}

def assign_ids(boxes):
    """Return {box_index: aruco_id} with minimum total squared distance to quad centers."""
    best_cost = float("inf")
    best_perm = BOX_IDS
    for perm in permutations(BOX_IDS):
        cost = sum(
            (boxes[i][1] - QUAD_CENTERS[perm[i]][0]) ** 2 +
            (boxes[i][2] - QUAD_CENTERS[perm[i]][1]) ** 2
            for i in range(len(boxes))
        )
        if cost < best_cost:
            best_cost = cost
            best_perm = perm
    return {i: best_perm[i] for i in range(len(boxes))}

CORNER_DST = {
    0: [0,         0        ],
    1: [WARP_SIZE, 0        ],
    2: [WARP_SIZE, WARP_SIZE],
    3: [0,         WARP_SIZE],
}

ID_COLOR = {10: (0,165,255), 11: (255,100,0), 12: (0,200,200), 13: (180,0,255)}
LABEL    = {10: "10 TL", 11: "11 TR", 12: "12 BR", 13: "13 BL"}


def marker_center(entry):
    pts = entry[0]
    return pts[:, 0].mean(), pts[:, 1].mean()


def compute_homography(corners, ids):
    mapping = {}
    for i, mid in enumerate(ids.flatten()):
        if mid in CORNER_ARUCO_IDS:
            mapping[mid] = list(marker_center(corners[i]))
    if len(mapping) < 4:
        return None
    src = np.array([mapping[c] for c in CORNER_ARUCO_IDS], dtype=np.float32)
    dst = np.array([CORNER_DST[c] for c in CORNER_ARUCO_IDS], dtype=np.float32)
    H, _ = cv2.findHomography(src, dst)
    return H


def detect_boxes(warped):
    hsv  = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(BOX_HSV_LO), np.array(BOX_HSV_HI))
    k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for cnt in contours:
        if cv2.contourArea(cnt) >= BOX_MIN_AREA:
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
                boxes.append((cnt, cx, cy))
    return boxes


def save_positions(positions):
    """Rewrite BOX_POSITIONS in arena_config.py with detected values."""
    lines = ["BOX_POSITIONS = {\n"]
    for aid, (x_m, y_m) in sorted(positions.items()):
        lines.append(f"    {aid}: ({x_m:.3f}, {y_m:.3f}),\n")
    lines.append("}\n")
    block = "".join(lines)

    with open(ARENA_CFG, "r") as f:
        src = f.read()

    # Replace existing BOX_POSITIONS block if present, else append
    pattern = r"BOX_POSITIONS\s*=\s*\{[^}]*\}\n?"
    if re.search(pattern, src):
        src = re.sub(pattern, block, src)
    else:
        src = src.rstrip("\n") + "\n\n" + block

    with open(ARENA_CFG, "w") as f:
        f.write(src)
    print(f"\nSaved BOX_POSITIONS to arena_config.py:")
    for aid, (x, y) in sorted(positions.items()):
        print(f"  {aid}: ({x:.3f}m, {y:.3f}m)  {LABEL[aid]}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", required=True, help="MJPEG stream URL")
    args = parser.parse_args()

    if not hasattr(cv2, "aruco"):
        print("ERROR: cv2.aruco missing — install opencv-contrib-python")
        sys.exit(1)

    aruco_dict   = cv2.aruco.getPredefinedDictionary(DICT_ID)
    aruco_params = cv2.aruco.DetectorParameters()
    detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    cap = cv2.VideoCapture(args.url)
    if not cap.isOpened():
        print(f"ERROR: cannot open {args.url}")
        sys.exit(1)

    print("Waiting for corner markers (IDs 0-3)…  Press [s] to save, [q] to quit.")
    print(f"Using: HSV_LO={BOX_HSV_LO}  HSV_HI={BOX_HSV_HI}  MIN_AREA={BOX_MIN_AREA}")

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
        boxes_detected = []

        if ids is not None:
            new_H = compute_homography(corners, ids)
            if new_H is not None:
                H = new_H

        if H is not None:
            warped = cv2.warpPerspective(frame, H, (WARP_SIZE, WARP_SIZE))
            boxes_detected = detect_boxes(warped)

            if len(boxes_detected) == 4:
                idx_to_id = assign_ids(boxes_detected)
            else:
                idx_to_id = {}

            for i, (cnt, cx, cy) in enumerate(boxes_detected):
                aid   = idx_to_id.get(i)
                color = ID_COLOR[aid] if aid else (100, 100, 100)
                label = LABEL[aid]    if aid else "?"
                cv2.drawContours(warped, [cnt], -1, color, 2)
                cv2.circle(warped, (int(cx), int(cy)), 6, color, -1)
                x_m = (cx / PX_PER_CM) / 100.0
                y_m = (cy / PX_PER_CM) / 100.0
                cv2.putText(warped, f"ID {label}",
                            (int(cx) + 8, int(cy) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                cv2.putText(warped, f"{x_m:.2f}m, {y_m:.2f}m",
                            (int(cx) + 8, int(cy) + 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        count = len(boxes_detected)
        status_color = (0, 255, 0) if count == 4 else (0, 0, 255)
        cv2.putText(warped, f"boxes: {count}/4", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        if count == 4:
            cv2.putText(warped, "press [s] to save positions", (10, 54),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        fps_counter += 1
        elapsed = time.time() - t0
        if elapsed >= 1.0:
            fps_display = fps_counter / elapsed
            fps_counter = 0
            t0 = time.time()
        cv2.putText(warped, f"{fps_display:.1f} fps", (10, WARP_SIZE - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1)

        cv2.imshow("box_detect", warped)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        if key == ord("s"):
            if count != 4:
                print(f"Need 4 boxes, only {count} detected — adjust view and retry.")
            else:
                mapping = assign_ids(boxes_detected)
                positions = {}
                for i, (_, cx, cy) in enumerate(boxes_detected):
                    aid = mapping[i]
                    positions[aid] = ((cx / PX_PER_CM) / 100.0,
                                      (cy / PX_PER_CM) / 100.0)
                save_positions(positions)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
