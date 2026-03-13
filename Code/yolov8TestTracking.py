## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 RealSense, Inc. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import time

showImg = True
model = YOLO("ConvertYOLO/best.pt")
cameraFOV = 69
imageWidth = 640
imageHeight = 480
imageCenter = imageWidth/2
fx = imageCenter/np.tan(np.radians(cameraFOV/2)) 
tracking = False
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
counter = 0
# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
start = time.time()
try:
    while True:


        # Wait for a coherent pair of frames: depth and color
        try:
            frames = pipeline.wait_for_frames()
        except RuntimeError:
            print(f"Frame Timeout")
            continue
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if counter > 10:
            tracking = False
            counter = 0

        if tracking == False:
            results = model.predict(source=color_image,conf=0.70, verbose=False)
            annoted = results[0].plot()

            detections = results[0].boxes
            if len(detections) > 0:
                best = max(detections, key=lambda b: float(b.conf[0]))
                cx, cy, w, h = best.xywh[0].tolist()
                x = cx - w/2.0
                y = cy - h/2.0
                bbox = (x, y, w, h)
                tracker = cv2.legacy.TrackerCSRT_create()
                tracker.init(color_image, bbox)
                counter = 0
                tracking = True
        else:
            success, bbox = tracker.update(color_image)
            if success:
                x, y, w, h = map(int, bbox)
                cx = x + w/2
                cy = y + h/2
                if showImg:
                    cv2.rectangle(color_image, (int(x),int(y)), (int(x+w), int(y+h)), (0,255,0), 2)
                offset = cx - imageCenter
                angle_rad = np.atan(offset/fx)
                angle_deg = np.rad2deg(angle_rad)
                cxi = int(np.clip(x,0, imageWidth -1))
                cyi =int(np.clip(y,0, imageHeight -1))
                print(f"Distance to person: {depth_frame.get_distance(int(cxi),int(cyi))} [m]")
                print(f"Angle to person: {angle_deg} [deg]")
                counter += 1
            else:
                cv2.putText(color_image, "Tracking lost", (20,60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                tracking = False

        # Show images
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', images)
        if not tracking:
            cv2.imshow('Yolo', annoted)
        else:
            cv2.imshow("Tracking", color_image)
        cv2.waitKey(1)
        end = time.time()
        frameTime = end - start
        print(f"Frame Time: {1/frameTime}")
        start = end

finally:

    # Stop streaming
    pipeline.stop()