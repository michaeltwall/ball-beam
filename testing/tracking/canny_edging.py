import cv2 as cv
import numpy as np

img = cv.imread('testing/tracking/messi5.jpg', cv.IMREAD_GRAYSCALE)

def nothing(x):
    pass

cv.namedWindow('image')
cv.createTrackbar('Lower', 'image', 100, 255, nothing)
cv.createTrackbar('Upper', 'image', 200, 255, nothing)

while True:
    upper = cv.getTrackbarPos('Upper', 'image')
    lower = cv.getTrackbarPos('Lower', 'image')

    edges = cv.Canny(img, lower, upper)

    cv.imshow('image', edges)

    k = cv.waitKey(30) & 0xFF
    if k == 27:  # Escape key
        break

cv.destroyAllWindows()