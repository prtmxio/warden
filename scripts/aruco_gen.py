import cv2
import numpy as np
import os

# Must match DICT_ID in every other script and vision_node — changing this breaks all detection
DICT_ID   = cv2.aruco.DICT_4X4_50
MARKER_PX = 400
BORDER_PX = 50
IDS = [0, 1, 2, 3, 5, 10, 11, 12, 13]

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "arena", "markers")
os.makedirs(OUTPUT_DIR, exist_ok=True)

aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)
total = MARKER_PX + 2 * BORDER_PX

for mid in IDS:
    img = cv2.aruco.generateImageMarker(aruco_dict, mid, MARKER_PX)
    canvas = np.ones((total, total), dtype=np.uint8) * 255
    canvas[BORDER_PX:BORDER_PX + MARKER_PX, BORDER_PX:BORDER_PX + MARKER_PX] = img
    path = os.path.join(OUTPUT_DIR, f"id_{mid}.png")
    cv2.imwrite(path, canvas)
    print(f"  wrote {path}")

print(f"\nDone — {len(IDS)} markers in {OUTPUT_DIR}")
print("Print each at ~8x8 cm and laminate.")
print("  IDs 0-3   → arena corners, face UP")
print("  ID  5     → bot top on 5cm standoff, face UP")
print("  IDs 10-13 → box side face, permanent")
