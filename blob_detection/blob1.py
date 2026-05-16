import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import numpy as np
import cv2 as cv
import undistort as ud
from collections import deque

def nothing(x):
    pass

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FPS, 60.0) 
cap.set(cv.CAP_PROP_FRAME_WIDTH,1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,1080)

cv.namedWindow('display', cv.WINDOW_NORMAL)
cv.resizeWindow('display', 640,400)

cv.createTrackbar('hue_lower', 'display', 5, 179, nothing)
cv.createTrackbar('hue_upper', 'display', 20, 179, nothing)
# cv.createTrackbar('min_sat', 'display', 80, 255, nothing)
# cv.createTrackbar('min_value', 'display', 80, 255, nothing)

# store recent positions
history_x = deque(maxlen=300)
history_y = deque(maxlen=300)

crosshair_length = 100

kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (11,11))

k = cv.KalmanFilter(4, 2)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame")
        break

    frame = ud.undistort(frame)

    
    lower_hue = cv.getTrackbarPos('hue_lower', 'display')
    upper_hue = cv.getTrackbarPos('hue_upper', 'display')
    # min_sat = cv.getTrackbarPos('min_sat', 'display')
    # min_value = cv.getTrackbarPos('min_value', 'display')


    # define range of blue color in HSV
    lower = np.array([lower_hue,140,140])
    upper = np.array([upper_hue,255,255])

    blur_frame = cv.GaussianBlur(frame, (9,9), 0)
    hsv = cv.cvtColor(blur_frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)
    # opening = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

    M = cv.moments(mask)
    if M["m00"] > 0:
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        history_x.append(cx)
        history_y.append(cy)

        cv.putText(frame, f"X: {cx:.02f} Y: {cy:.02f}", (int(cx) + 10, int(cy) - 10),cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cx = round(cx)
        cy = round(cy)
        cv.circle(frame, (cx, cy), 8, (0, 0, 255), -1)      #center dot
        cv.line(frame,(cx-crosshair_length, cy),(cx+crosshair_length, cy),(0,0,0),2)
        cv.line(frame,(cx, cy-crosshair_length),(cx, cy+crosshair_length),(0,0,0),2)

    cv.imshow('display', mask)
    cv.imshow('display2', frame)
    if len(history_x) > 30:
        x_std = np.std(history_x)
        y_std = np.std(history_y)

        print(f"Jitter X: ±{x_std:.4f} px | "
                f"Jitter Y: ±{y_std:.4f} px")

    if cv.waitKey(1) == 27:
        break

cap.release()
cv.destroyAllWindows()