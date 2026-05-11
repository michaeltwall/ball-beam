import numpy as np
import cv2 as cv

def nothing(x):
    pass

drawing = False 
r,g,b = 0,0,0
rad = 3

def draw_circle(event,x,y,flags,param):
    global drawing,img,img_temp
 
    if event == cv.EVENT_LBUTTONDOWN:
        drawing = True
        img_temp = img.copy()
 
    elif event == cv.EVENT_MOUSEMOVE:        
        if drawing == True: 
            img_temp = img.copy()
        else:
            img = img_temp.copy()
        cv.circle(img,(x,y),rad,(b,g,r),-1)
 
    elif event == cv.EVENT_LBUTTONUP:
        drawing = False
        cv.circle(img,(x,y),rad,(b,g,r),-1)

img = np.zeros((300,650,3),np.uint8)
img_temp = img.copy()
cv.namedWindow('image')
cv.setMouseCallback('image',draw_circle)

cv.createTrackbar('R','image',255,255,nothing)
cv.createTrackbar('G','image',0,255,nothing)
cv.createTrackbar('B','image',0,255,nothing)

radius = 'Brush Radius'
cv.createTrackbar(radius,'image',5,100,nothing)

while(1):
    cv.imshow('image',img)
    k = cv.waitKey(1) & 0xFF
    if k == 27:
        break

    r = cv.getTrackbarPos('R','image')
    g = cv.getTrackbarPos('G','image')
    b = cv.getTrackbarPos('B','image')
    rad = cv.getTrackbarPos(radius,'image')
 
cv.destroyAllWindows() 