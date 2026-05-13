import cv2 as cv


cam_file = cv.FileStorage("calibration/cam.yml", cv.FILE_STORAGE_READ)
w = int(cam_file.getNode("image_width").real())
h = int(cam_file.getNode("image_height").real())
mtx = cam_file.getNode('camera_matrix').mat()
dist = cam_file.getNode('distortion_coefficients').mat()

newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
x, y, w, h = roi

cap = cv.VideoCapture(0)    #external webcam is 1, builtin/default is 0 (or -1)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    dst = cv.undistort(frame, mtx, dist, None, newcameramtx)
    dst = dst[y:y+h, x:x+w]

    # Display the resulting frame
    cv.imshow('frame', frame)
    cv.imshow('undistort', dst)
    if cv.waitKey(1) == 27:
        break