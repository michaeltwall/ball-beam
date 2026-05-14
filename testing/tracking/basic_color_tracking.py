'''
OpenCV Example from: https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
'''

import cv2 as cv
import numpy as np

def nothing(x):
    pass

cap = cv.VideoCapture(0)
range = 20

cv.namedWindow('trackbar')
cv.createTrackbar('Hue','trackbar',110,179-10,nothing)

while(1):

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    hue = cv.getTrackbarPos('Hue','trackbar')

    # define range of blue color in HSV
    lower_blue = np.array([hue,50,50])
    upper_blue = np.array([hue+range,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    res = cv.bitwise_and(frame,frame, mask= mask)

    cv.imshow('frame',frame)
    cv.imshow('mask',mask)
    cv.imshow('res',res)
    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break

cv.destroyAllWindows()