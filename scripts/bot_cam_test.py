import cv2
import sys
import matplotlib.pyplot as plt

S3_STREAM_URL = "http://10.200.124.103:81/stream"

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params     = cv2.aruco.DetectorParameters()
detector   = cv2.aruco.ArucoDetector(aruco_dict, params)

cap = cv2.VideoCapture(S3_STREAM_URL, cv2.CAP_FFMPEG)
if not cap.isOpened():
    print(f"Cannot open stream: {S3_STREAM_URL}")
    sys.exit(1)

print("Stream open. Hold a box ArUco (IDs 10-13) in front of the camera. Close window to quit.")

plt.ion()
fig, ax = plt.subplots(figsize=(6, 5))
ax.axis("off")
im_handle = None

while plt.get_fignums():
    ret, frame = cap.read()
    if not ret:
        print("Frame grab failed")
        break

    corners, ids, _ = detector.detectMarkers(frame)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for id_val in ids.flatten():
            print(f"ArUco ID: {id_val}")

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    if im_handle is None:
        im_handle = ax.imshow(rgb)
    else:
        im_handle.set_data(rgb)

    fig.canvas.flush_events()
    plt.pause(0.01)

cap.release()
plt.close()
