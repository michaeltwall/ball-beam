import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import numpy as np
import cv2 as cv
import undistort as ud

def nothing(x):
    pass

def find_moment(mask):
    M = cv.moments(mask)
    if M["m00"] > 0:
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
    else:
        cx = cy = None
    return cx, cy

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FPS, 60.0) 
cap.set(cv.CAP_PROP_FRAME_WIDTH,1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,1080)

cv.namedWindow('display', cv.WINDOW_NORMAL)
cv.resizeWindow('display', 640,400)

cv.createTrackbar('hue_lower', 'display', 80, 179, nothing)
cv.createTrackbar('hue_upper', 'display', 115, 179, nothing)
cv.createTrackbar('min_sat', 'display', 140, 255, nothing)
cv.createTrackbar('min_value', 'display', 64, 255, nothing)

lower_ball = np.array([5,180,180])
upper_ball = np.array([20,255,255])

crosshair_length = 100

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

    blur_frame = cv.GaussianBlur(frame, (5,5), 0)
    hsv = cv.cvtColor(blur_frame, cv.COLOR_BGR2HSV)
    
    ball_mask = cv.inRange(hsv, lower_ball, upper_ball)

    cx, cy = find_moment(ball_mask)
    if cx is not None:
        cv.putText(frame, f"X: {cx:.02f} Y: {cy:.02f}", (int(cx) + 10, int(cy) - 10),cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cx, cy = round(cx), round(cy)
        cv.circle(frame, (cx, cy), 8, (0, 0, 255), -1)      #center dot
        cv.line(frame,(cx-crosshair_length, cy),(cx+crosshair_length, cy),(0,0,0),2)
        cv.line(frame,(cx, cy-crosshair_length),(cx, cy+crosshair_length),(0,0,0),2)

    lower_hue = cv.getTrackbarPos('hue_lower', 'display')
    upper_hue = cv.getTrackbarPos('hue_upper', 'display')
    min_sat = cv.getTrackbarPos('min_sat', 'display')
    min_value = cv.getTrackbarPos('min_value', 'display')

    # define range of blue color in HSV
    lower_beam = np.array([lower_hue,min_sat,min_value])
    upper_beam = np.array([upper_hue,255,255])
    beam_mask = cv.inRange(hsv, lower_beam, upper_beam)

    if beam_mask is not None:
        beam_mask_left, beam_mask_right = np.array_split(beam_mask,2,1)

        xb0, yb0 = find_moment(beam_mask_left)
        xb1, yb1 = find_moment(beam_mask_right)

        if xb0 is not None:
            xb0, yb0 = round(xb0), round(yb0)
            cv.circle(frame, (xb0, yb0), 8, (0, 0, 255), -1)
        if xb1 is not None:
            xb1 += beam_mask_left.shape[1]
            xb1, yb1 = round(xb1), round(yb1)
            cv.circle(frame, (xb1, yb1), 8, (0, 0, 255), -1)
        if xb0 is not None and xb1 is not None:
            cv.line(frame, (xb0, yb0), (xb1, yb1), (255, 0, 0), 2)

    cv.imshow('display', beam_mask)
    cv.imshow('display2', frame)

    if cv.waitKey(1) == 27:
        break

cap.release()
cv.destroyAllWindows()