import pyrealsense2 as rs
import numpy as np
import cv2
import csv
import os
import time
from datetime import datetime

# =========================
# Setup
# =========================
CSV_PATH = "5meter20vinkel.csv"

LOG_EVERY_N_FRAMES = 10      # logging every 10. frame

clicked_point = None
logging_active = False
frame_count = 0

# =========================
# CSV funktions
# =========================
def ensure_csv_with_header(path):
    if not os.path.exists(path):
        with open(path, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["pixel_x", "pixel_y", "distance_m", "angle_deg"])

def append_row(path, row):
    with open(path, mode="a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(row)

# =========================
# Mouse callback
# =========================
def mouse_callback(event, x, y, flags, param):
    global clicked_point, logging_active
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)
        logging_active = True
        print(f"Start logging på pixel: {clicked_point}")
    elif event == cv2.EVENT_RBUTTONDOWN:
        logging_active = False
        print("Stopper logging")

# =========================
# RealSense setup
# =========================
pipeline = rs.pipeline()
config = rs.config()

imageWidth = 640
imageHeight = 480

config.enable_stream(rs.stream.depth, imageWidth, imageHeight, rs.format.z16, 30)
config.enable_stream(rs.stream.color, imageWidth, imageHeight, rs.format.bgr8, 30)

pipeline_profile = pipeline.start(config)

align = rs.align(rs.stream.color)

cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback("RealSense", mouse_callback)
ensure_csv_with_header(CSV_PATH)

print("Klar. Venstreklik for at starte logging. Højreklik for at stoppe.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)

        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Getting the intrinsics values from the camera
        intr = color_frame.profile.as_video_stream_profile().intrinsics
        fx = intr.fx
        cx = intr.ppx

        color_image = np.asanyarray(color_frame.get_data())

        # Draw the chosen point
        if clicked_point is not None:
            cv2.circle(color_image, clicked_point, 5, (0, 0, 255), -1)
            cv2.putText(color_image, "LOG" if logging_active else "PAUSE",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if logging_active else (0, 255, 255), 2)

        # Logging-styring
        should_log = False
        frame_count += 1

        if logging_active and clicked_point is not None:
            if frame_count % LOG_EVERY_N_FRAMES == 0:
                should_log = True

        # If we should log
        if should_log:
            px, py = clicked_point

            distance = depth_frame.get_distance(px, py)

            # Angle from intrinsics
            px_offset = px - cx
            angle = np.degrees(np.arctan(px_offset / fx))

            print(f"px={px} py={py}  distance={distance:.3f} m  angle={angle:.2f} deg")

            append_row(CSV_PATH, [
                int(px), int(py),
                round(distance, 3),
                round(angle, 3)
            ])

        # =========================
        # Draw angle lines (-15°, 0°, +15°) via intrinsics
        # =========================

        angles_to_draw = [-15, 0, 15]

        for a in angles_to_draw:
            offset_px = np.tan(np.radians(a)) * fx   
            x_pos = int(cx + offset_px)              

            if 0 <= x_pos < imageWidth:
                color = (0, 255, 0) if a == 0 else (255, 0, 0)
                cv2.line(color_image, (x_pos, 0), (x_pos, imageHeight), color, 2)
                cv2.putText(color_image, f"{a} deg", (x_pos + 5, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        cv2.imshow("RealSense", color_image)

        if cv2.waitKey(1) == 27:  # ESC
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()