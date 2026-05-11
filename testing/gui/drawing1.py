import numpy as np
import cv2 as cv

drawing = False # true if mouse is pressed
mode = True # if True, draw rectangle. Press 'm' to toggle to curve
ix,iy = -1,-1
 
# mouse callback function
def draw_circle(event,x,y,flags,param):
    global ix,iy,drawing,mode,img,img_temp
 
    if event == cv.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y
        img_temp = img.copy()
 
    elif event == cv.EVENT_MOUSEMOVE:
        if mode == True:
            if drawing == True:
                img = img_temp.copy()
                cv.rectangle(img,(ix,iy),(x,y),(0,255,0),3)
        else:           
            if drawing == True: 
                img_temp = img.copy()
            else:
                img = img_temp.copy()
            cv.circle(img,(x,y),5,(0,0,255),-1)
 
    elif event == cv.EVENT_LBUTTONUP:
        drawing = False
        if mode == True:
            img = img_temp.copy()
            cv.rectangle(img,(ix,iy),(x,y),(0,255,0),3)
        else:
            cv.circle(img,(x,y),5,(0,0,255),-1)

img = np.zeros((512,512,3), np.uint8)
img_temp = img.copy()
cv.namedWindow('image')
cv.setMouseCallback('image',draw_circle)
 
while(1):
    cv.imshow('image',img)
    k = cv.waitKey(1) & 0xFF
    if k == ord('m'):
        mode = not mode
        img = img_temp.copy()
    elif k == 27:
        break
 
cv.destroyAllWindows()