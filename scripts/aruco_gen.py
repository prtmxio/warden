"""
Generate ArUco marker PNGs for the Loco Bot arena.
IDs used:  0,1,2,3 (floor corners), 5 (bot top), 10,11,12,13 (box sides)
Output:    arena/markers/id_<n>.png  — each 500x500 px with a 50px white border
Run from:  project root  →  python scripts/aruco_gen.py
"""

import cv2
import numpy as np
import os

DICT_ID  = cv2.aruco.DICT_4X4_50
MARKER_PX = 400   # inner marker size
BORDER_PX = 50    # white border around marker
IDS = [0, 1, 2, 3, 5, 10, 11, 12, 13]

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "arena", "markers")
os.makedirs(OUTPUT_DIR, exist_ok=True)

aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)

total = MARKER_PX + 2 * BORDER_PX  # 500 px per image

for mid in IDS:
    img = cv2.aruco.generateImageMarker(aruco_dict, mid, MARKER_PX)
    canvas = np.ones((total, total), dtype=np.uint8) * 255
    canvas[BORDER_PX:BORDER_PX + MARKER_PX, BORDER_PX:BORDER_PX + MARKER_PX] = img
    path = os.path.join(OUTPUT_DIR, f"id_{mid}.png")
    cv2.imwrite(path, canvas)
    print(f"  wrote {path}")

print(f"\nDone — {len(IDS)} markers in {OUTPUT_DIR}")
print("Print each at ~8x8 cm and laminate.")
print("Placement:")
print("  IDs 0-3  → flat at 4 corners of arena (face UP)")
print("  ID  5    → top of bot on 5 cm standoff (face UP)")
print("  IDs 10-13→ one per box, taped to side face (permanent)")
