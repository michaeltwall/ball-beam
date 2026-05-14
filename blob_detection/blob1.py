import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import numpy as np
import cv2 as cv
from calibration/undistort.py import undistort

def nothing(x):
    pass

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FPS, 60.0) 
cap.set(cv.CAP_PROP_FRAME_WIDTH,1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,1080) 

cv.namedWindow('display', cv.WINDOW_NORMAL)
cv.resizeWindow('display', 640,360)

cv.createTrackbar('hue_lower', 'display', 5, 179, nothing)
cv.createTrackbar('hue_upper', 'display', 20, 179, nothing)
# cv.createTrackbar('minRadius', 'display', 75, 200, nothing)
# cv.createTrackbar('maxRadius', 'display', 120, 200, nothing)
param1 = 100
param2 = 15
minRadius = 75
maxRadius = 120

kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (11,11))

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame")
        break

    frame = undistort(frame)

    
    lower_hue = cv.getTrackbarPos('hue_lower', 'display')
    upper_hue = cv.getTrackbarPos('hue_upper', 'display')

    # define range of blue color in HSV
    lower = np.array([lower_hue,80,80])
    upper = np.array([upper_hue,255,255])

    blur_frame = cv.GaussianBlur(frame, (9,9), 0)
    hsv = cv.cvtColor(blur_frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    opening = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    edge_blur = cv.GaussianBlur(opening, (5,5), 0)
    _, thresh = cv.threshold(edge_blur, 127, 255, cv.THRESH_BINARY)

    contours, _ = cv.findContours(opening, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv.contourArea)
        ((x, y), r) = cv.minEnclosingCircle(c)
        
        if r > minRadius:  # filter out tiny noise contours
            cx, cy, cr = int(x), int(y), int(r)
            cv.circle(frame, (cx, cy), cr, (0, 255, 0), 2)      # outline
            cv.circle(frame, (cx, cy), 4, (0, 0, 255), -1)      # center dot

    cv.imshow('display', opening)
    cv.imshow('display2', frame)

    if cv.waitKey(1) == 27:
        break

cap.release()
cv.destroyAllWindows()