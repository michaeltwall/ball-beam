import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import numpy as np
import cv2 as cv
from filterpy.kalman import KalmanFilter

cap = cv.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

def nothing(x):
    pass

# Separate window for tuning Kalman parameters
cv.namedWindow("Tuning", cv.WINDOW_NORMAL)

# --- Trackbars for R (Measurement Noise) ---
# Higher R means you trust the sensor LESS
cv.createTrackbar("R_pos", "Tuning", 5, 200, nothing)    # x, y noise
cv.createTrackbar("R_rad", "Tuning", 500, 2000, nothing)   # radius noise

# --- Trackbars for Q (Process Noise) ---
# Higher Q means the object can change direction/speed more erratically
cv.createTrackbar("Q_pos", "Tuning", 1, 100, nothing)    # x, y, r uncertainty
cv.createTrackbar("Q_rad", "Tuning", 1, 100, nothing)
cv.createTrackbar("Q_vel", "Tuning", 1, 100, nothing)    # velocity uncertainty

# Kalman filter:
f = KalmanFilter (dim_x=6, dim_z=3)
#f.x:  x, y, r, vx, vy, vr

f.F = np.eye(6)
f.F[0,3] = f.F[1,4] = f.F[2,5] = 1
f.H = np.eye(3,6)
f.P = np.diag([1,1,1,1,1,1])*500

Z_meas = np.zeros([1,3])

while True:

    r_pos = cv.getTrackbarPos('R_pos', 'Tuning')
    r_rad = cv.getTrackbarPos('R_rad', 'Tuning')
    q_pos = cv.getTrackbarPos('Q_pos', 'Tuning') * 0.01
    q_rad = cv.getTrackbarPos('Q_rad', 'Tuning') * 0.01
    q_vel = cv.getTrackbarPos('Q_vel', 'Tuning') * 0.01

    # 2. Update R Matrix (Measurement Noise)
    # R is 3x3: [x, y, radius]
    f.R = np.diag([r_pos, r_pos, r_rad])

    # 3. Update Q Matrix (Process Noise)
    # Q is 6x6: [x, y, r, vx, vy, vr]
    # We apply q_pos to positions and q_vel to velocities
    f.Q = np.diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel])

    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame")
        break

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray,(5,5),0)
    _,thresh = cv.threshold(blur,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)

    rows = thresh.shape[0]
    circles = cv.HoughCircles(thresh, cv.HOUGH_GRADIENT, 1, rows / 8,
                               param1=100, param2=15,
                               minRadius=25, maxRadius=50)
    
    f.predict()
    
    if circles is not None:
        circles = np.uint16(np.around(circles))
        Z_meas = circles[0,0,:]  # measures the first (most likely) circle
        f.update(Z_meas)

    # circle center
    center = tuple(f.x[:2, 0].astype(int))
    cv.circle(frame, center, 1, (0, 100, 100), 3)
    # circle outline
    radius = f.x[2,0].astype(int)
    cv.circle(frame, center, radius, (255, 0, 255), 3)

    cv.imshow("threshold", thresh)
    cv.imshow("circle detect", frame)

    if cv.waitKey(1) == 27:
        break

cap.release()
cv.destroyAllWindows()