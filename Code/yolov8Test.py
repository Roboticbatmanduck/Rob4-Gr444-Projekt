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
import argparse

def meanDistance(depth_frame,cx,cy, imageWidth, imageHeight):
    n=3 #3x3
    lst = []
    for i in range(-1, n-1):
        for j in range(-1, n-1):
            d = depth_frame.get_distance(int(np.clip(cx+i, 0, imageWidth-1)),int(np.clip(cy+j, 0, imageHeight - 1)))
            if d > 0:
                lst.append(d)
    if len(lst) == 0:
        return 0
    mean = sum(lst)/len(lst)
    return np.round(mean,3)

def main():
    model = YOLO("best.engine", task="detect")
    cameraFOV = 69
    imageWidth = int(640)
    imageHeight = int(480)
    imageCenter = imageWidth/2
    fx = imageCenter/np.tan(np.radians(cameraFOV/2)) 
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="Kør uden GUI")
    args = parser.parse_args()
    headless = args.headless
    print(args)

    
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
    align = rs.align(rs.stream.color)
    start = time.time()
    try:
        while True:


            # Wait for a coherent pair of frames: depth and color
            try:
                frames = pipeline.wait_for_frames()
            except RuntimeError:
                print(f"Frame Timeout")
                continue
            frames = align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            results = model.predict(source=color_image,conf=0.70, verbose=False)

            detections = results[0].boxes
            if len(detections) > 0:
                best = max(detections, key=lambda b: float(b.conf[0]))
                cx, cy, w, h = best.xywh[0].tolist()
                x = cx - w/2.0
                y = cy - h/2.0
                if not headless:
                    cv2.rectangle(color_image, (int(x),int(y)), (int(x+w), int(y+h)), (0,255,0), 2)
                    cv2.circle(color_image, (int(cx),int(cy)), 1, (0,0,255), -1)
                offset = cx - imageCenter
                angle_rad = np.arctan(offset/fx) 
                angle_deg = np.rad2deg(angle_rad)
                print(f"Distance to person: {meanDistance(depth_frame, cx, cy, imageWidth, imageHeight)} [m]")
                print(f"Angle to person: {angle_deg} [deg]")
                

            if not headless:
                annoted = results[0].plot()
                cv2.imshow('Yolo', annoted)
                cv2.waitKey(1)
            end = time.time()
            frameTime = end - start
            print(f"Frame Time: {1/frameTime}")
            start = end

    finally:

        # Stop streaming
        pipeline.stop()

if __name__ == "__main__":
    main()

    
    
