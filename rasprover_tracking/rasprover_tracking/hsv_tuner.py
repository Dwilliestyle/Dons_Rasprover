#!/usr/bin/env python3
"""
hsv_tuner.py
------------
A standalone OpenCV utility to find the right HSV values for your
tracking target BEFORE deploying to the RasPRover.

Run this ON YOUR DESKTOP (not the robot) with the webcam attached,
or SSH with X forwarding from the robot.

Usage:
    python3 hsv_tuner.py          # uses camera index 0
    python3 hsv_tuner.py 1        # uses camera index 1

Controls:
    Drag the trackbars to isolate your object in the MASK window.
    Press 'p' to print the current values (copy into tracking_params.yaml)
    Press 'q' to quit.

This node is also registered as a ros2 run entry point, but it does
NOT use ROS2 — it runs as plain Python so you can use it without
a running ROS2 environment.
"""

import sys
import cv2
import numpy as np


def nothing(x):
    pass


def main(args=None):
    cam_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    cap = cv2.VideoCapture(cam_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f'ERROR: Could not open camera at index {cam_idx}')
        sys.exit(1)

    cv2.namedWindow('HSV Tuner', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mask',      cv2.WINDOW_NORMAL)
    cv2.namedWindow('Result',    cv2.WINDOW_NORMAL)

    # Trackbars
    cv2.createTrackbar('H Min', 'HSV Tuner',   0, 179, nothing)
    cv2.createTrackbar('H Max', 'HSV Tuner',  30, 179, nothing)
    cv2.createTrackbar('S Min', 'HSV Tuner', 100, 255, nothing)
    cv2.createTrackbar('S Max', 'HSV Tuner', 255, 255, nothing)
    cv2.createTrackbar('V Min', 'HSV Tuner',  50, 255, nothing)
    cv2.createTrackbar('V Max', 'HSV Tuner', 255, 255, nothing)

    print('HSV Tuner running.')
    print('  Hold your object in front of the camera.')
    print('  Adjust sliders until only your object is white in the Mask window.')
    print("  Press 'p' to print values, 'q' to quit.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print('Failed to read frame.')
            break

        h_min = cv2.getTrackbarPos('H Min', 'HSV Tuner')
        h_max = cv2.getTrackbarPos('H Max', 'HSV Tuner')
        s_min = cv2.getTrackbarPos('S Min', 'HSV Tuner')
        s_max = cv2.getTrackbarPos('S Max', 'HSV Tuner')
        v_min = cv2.getTrackbarPos('V Min', 'HSV Tuner')
        v_max = cv2.getTrackbarPos('V Max', 'HSV Tuner')

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask  = cv2.inRange(hsv, lower, upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

        result = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow('HSV Tuner', frame)
        cv2.imshow('Mask',      mask)
        cv2.imshow('Result',    result)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            print('--- Copy these into tracking_params.yaml ---')
            print(f'    h_min: {h_min}')
            print(f'    h_max: {h_max}')
            print(f'    s_min: {s_min}')
            print(f'    s_max: {s_max}')
            print(f'    v_min: {v_min}')
            print(f'    v_max: {v_max}')
            print('---------------------------------------------')

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()