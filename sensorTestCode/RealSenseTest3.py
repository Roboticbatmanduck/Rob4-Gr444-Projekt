import pyrealsense2 as rs
import numpy as np
import cv2
import csv
import os
import time
from datetime import datetime

# =========================
# Indstillinger / konstanter
# =========================
CSV_PATH = "5meter30vinkel.csv"

# Logging-frekvens (vælg én af strategierne)
LOG_EVERY_N_FRAMES = 10        # log hver N'te frame (1 = hver frame)
MIN_INTERVAL_SEC = 0.0        # alternativt tidsbaseret throttling (0.0 = ingen throttling)

# Kamera/FOV-parametre
cameraFOV = 69  # vandret FOV i grader (bruges kun til vinkelberegning)
imageWidth = int(640)
imageHeight = int(480)
imageCenter = imageWidth / 2
fx_from_fov = imageCenter / np.tan(np.radians(cameraFOV / 2))  # estimat af fx ud fra FOV (kun til vinkel)

# =========================
# Tilstand
# =========================
clicked_point = None          # seneste valgte punkt (px, py)
logging_active = False        # om vi logger kontinuerligt
frame_count = 0
next_log_time = 0.0

# =========================
# Hjælpefunktioner
# =========================
def meanDistance(depth_frame, cx, cy, imageWidth, imageHeight, n=3):
    """Robust middelværdi af dybde i et n×n vindue centreret i (cx, cy)"""
    half = n // 2
    vals = []
    for j in range(-half, half + 1):
        for i in range(-half, half + 1):
            x = int(np.clip(cx + i, 0, imageWidth - 1))
            y = int(np.clip(cy + j, 0, imageHeight - 1))
            d = depth_frame.get_distance(x, y)
            if d > 0:
                vals.append(d)
    if not vals:
        return 0.0
    return float(np.round(sum(vals) / len(vals), 3))

def ensure_csv_with_header(path):
    """Opret CSV med header, hvis den ikke findes"""
    if not os.path.exists(path):
        with open(path, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp",
                "pixel_x", "pixel_y",
                "depth_m_pixel",
                "depth_m_mean3x3",
                "angle_deg",
                "X_m", "Y_m", "Z_m"
            ])

def append_row(path, row):
    with open(path, mode="a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(row)

def mouse_callback(event, x, y, flags, param):
    global clicked_point, logging_active
    if event == cv2.EVENT_LBUTTONDOWN:
        # Venstreklik: vælg punkt og start logging
        clicked_point = (x, y)
        logging_active = True
        print(f"Start logging ved pixel: {clicked_point}")
    elif event == cv2.EVENT_RBUTTONDOWN:
        # Højreklik: stop logging (punktet beholdes visuelt)
        logging_active = False
        print("Stopper logging.")

# =========================
# RealSense opsætning
# =========================
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, imageWidth, imageHeight, rs.format.z16, 30)
config.enable_stream(rs.stream.color, imageWidth, imageHeight, rs.format.bgr8, 30)

pipeline_profile = pipeline.start(config)

# Align depth til color frame, så (x,y) fra farvebilledet matcher dybdens koordinater
align = rs.align(rs.stream.color)

# Opret vindue og CSV
cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback("RealSense", mouse_callback)
ensure_csv_with_header(CSV_PATH)

print(f"Logger klik til: {os.path.abspath(CSV_PATH)}")
print("Brug venstreklik for at starte logging på et punkt; højreklik for at stoppe. ESC for at afslutte.")

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Hent intrinsics fra det ALIGNede farve-stream (bruges til deprojektion til 3D)
        color_stream = color_frame.profile.as_video_stream_profile()
        intr = color_stream.get_intrinsics()  # intr.fx, intr.fy, intr.ppx, intr.ppy, intr.width, intr.height

        # Visualisér valgt punkt (hvis der er et)
        if clicked_point is not None:
            cv2.circle(color_image, clicked_point, 6, (0, 0, 255), -1)
            cv2.putText(color_image, "LOG" if logging_active else "PAUSE",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if logging_active else (0, 255, 255), 2)

        # Kontinuerlig logging hvis aktiv
        should_log = False
        frame_count += 1
        now = time.time()
        if logging_active and clicked_point is not None:
            # Frame-baseret throttling
            if LOG_EVERY_N_FRAMES <= 10 or (frame_count % LOG_EVERY_N_FRAMES == 0):
                should_log = True
            # Tidsbaseret throttling (overstyrer ikke frame-valget; brug begge til finstyring)
            if MIN_INTERVAL_SEC > 0.0 and now < next_log_time:
                should_log = False

        if should_log:
            px, py = clicked_point
            if 0 <= px < depth_image.shape[1] and 0 <= py < depth_image.shape[0]:
                # Dybder (meter)
                depth_value = float(depth_frame.get_distance(px, py))
                depth_mean3x3 = meanDistance(depth_frame, px, py, imageWidth, imageHeight, n=3)

                # Vinkel i forhold til billedcentrum (vandret), baseret på FOV-estimat
                offset = px - imageCenter
                angle_rad = np.arctan(offset / fx_from_fov)
                angle_deg = float(np.rad2deg(angle_rad))

                # 3D-koordinater (X, Y, Z) i kamerasystemet.
                Z = depth_mean3x3 if depth_mean3x3 > 0 else depth_value
                if Z > 0:
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, [float(px), float(py)], float(Z))
                else:
                    X, Y, Z = (0.0, 0.0, 0.0)

                # Vis nogle tal på billedet (valgfrit)
                cv2.putText(color_image, f"{depth_value:.3f} m",
                            (px + 10, py - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(color_image, f"{angle_deg:.3f} deg",
                            (px + 10, py + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # Log til konsol
                print(f"[{datetime.now().isoformat(timespec='seconds')}] "
                      f"px={px}, py={py}, depth={depth_value:.3f} m, mean3x3={depth_mean3x3:.3f} m, "
                      f"angle={angle_deg:.2f} deg, XYZ=({X:.3f}, {Y:.3f}, {Z:.3f})")

                # Gem til CSV
                append_row(CSV_PATH, [
                    datetime.now().isoformat(timespec="seconds"),
                    int(px), int(py),
                    round(depth_value, 3),
                    round(depth_mean3x3, 3),
                    round(angle_deg, 3),
                    round(X, 4), round(Y, 4), round(Z, 4)
                ])

                # Opdatér næste logtid hvis interval anvendes
                if MIN_INTERVAL_SEC > 0.0:
                    next_log_time = now + MIN_INTERVAL_SEC

        # Vis farvebillede
        cv2.imshow("RealSense", color_image)

        # Afslut med ESC
        if cv2.waitKey(1) == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
