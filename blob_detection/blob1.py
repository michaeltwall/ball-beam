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

def calc_proj(A, B, P):
    AB = B - A
    AP = P - A
    t = np.dot(AP, AB) / np.dot(AB, AB)
    return (A + t * AB)


cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FPS, 60.0) 
cap.set(cv.CAP_PROP_FRAME_WIDTH,1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,1080)

cv.namedWindow('mask', cv.WINDOW_NORMAL)
cv.resizeWindow('mask', 640,400)

cv.createTrackbar('hue_lower', 'mask', 20, 179, nothing)
cv.createTrackbar('hue_upper', 'mask', 35, 179, nothing)
cv.createTrackbar('min_sat', 'mask', 60, 255, nothing)
cv.createTrackbar('min_value', 'mask', 95, 255, nothing)

lower_ball = np.array([5,140,140])
upper_ball = np.array([20,255,255])


# lower_beam = np.array([25,60,95])
# upper_beam = np.array([35,255,255])

crosshair_length = 100

k = cv.KalmanFilter(4, 2)

dist = 0

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame")
        break
    
    lower_hue = cv.getTrackbarPos('hue_lower', 'mask')
    upper_hue = cv.getTrackbarPos('hue_upper', 'mask')
    min_sat = cv.getTrackbarPos('min_sat', 'mask')
    min_value = cv.getTrackbarPos('min_value', 'mask')

    # define range of blue color in HSV
    lower_beam = np.array([lower_hue,min_sat,min_value])
    upper_beam = np.array([upper_hue,255,255])

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

            A = np.array([xb0, yb0], dtype=np.float32)
            B = np.array([xb1, yb1], dtype=np.float32)
            P = np.array([cx, cy], dtype=np.float32)
            Q = calc_proj(A, B, P)
            dist = np.linalg.norm(Q - A)
            print(dist)

            cv.circle(frame, tuple(Q.astype(int)), 8, (0, 0, 255), -1)
            cv.line(frame, tuple(P.astype(int)),tuple(Q.astype(int)), (0, 255, 0), 2)



    cv.imshow('display', frame)
    cv.imshow('mask', beam_mask)

    if cv.waitKey(1) == 27:
        break

cap.release()
cv.destroyAllWindows()