import cv2 as cv
import numpy as np
import argparse
import sys

def main():
    # --- Command Line Arguments ---
    parser = argparse.ArgumentParser(description='Calibration using a ChArUco board')
    parser.add_argument('--w', type=int, default=7, help='Number of squares in X direction')
    parser.add_argument('--h', type=int, default=5, help='Number of squares in Y direction')
    parser.add_argument('--sl', type=float, default=.03514, help='Square side length (in meters)')
    parser.add_argument('--ml', type=float, default=.01778, help='Marker side length (in meters)')
    parser.add_argument('--d', type=int, default=10, help='Dictionary ID (0=4x4_50, etc.)')
    parser.add_argument('--outfile', type=str, default='calibration/cam.yml', help='Output file name')
    parser.add_argument('--v', type=str, help='Input from video file (if omitted, uses camera)')
    parser.add_argument('--ci', type=int, default=0, help='Camera ID')
    parser.add_argument('--rs', action='store_true', help='Apply refind strategy')
    parser.add_argument('--zt', action='store_true', help='Assume zero tangential distortion')
    parser.add_argument('--a', type=float, help='Fix aspect ratio (fx/fy)')
    parser.add_argument('--pc', action='store_true', help='Fix principal point at center')
    parser.add_argument('--sc', action='store_true', help='Show detected corners after calibration')

    args = parser.parse_args()

    # --- Setup Dictionary and Board ---
    # In newer OpenCV, access dictionaries via cv.aruco.getPredefinedDictionary
    dictionary = cv.aruco.getPredefinedDictionary(args.d)
    board = cv.aruco.CharucoBoard((args.w, args.h), args.sl, args.ml, dictionary)
    
    # Detector Parameters
    params = cv.aruco.DetectorParameters()
    charuco_params = cv.aruco.CharucoParameters()
    if args.rs:
        charuco_params.tryRefineMarkers = True
    
    detector = cv.aruco.CharucoDetector(board, charuco_params, params)

    # --- Video Input ---
    if args.v:
        cap = cv.VideoCapture(args.v)
        wait_time = 0
    else:
        cap = cv.VideoCapture(args.ci)
        wait_time = 10

    all_charuco_corners = []
    all_charuco_ids = []
    all_image_points = []
    all_object_points = []
    all_images = []
    image_size = None

    print("Press 'c' to capture, 'ESC' to start calibration.")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        image_copy = frame.copy()
        
        # Detect Board
        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(frame)

        # Draw results
        if marker_ids is not None:
            cv.aruco.drawDetectedMarkers(image_copy, marker_corners, marker_ids)
        
        if charuco_ids is not None and len(charuco_ids) > 3:
            cv.aruco.drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids)

        cv.putText(image_copy, "Press 'c' to add frame. 'ESC' to calibrate", 
                    (10, 25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        cv.imshow("Calibration", image_copy)
        key = cv.waitKey(wait_time) & 0xFF

        if key == 27: # ESC
            break
        
        if key == ord('c') and charuco_ids is not None and len(charuco_ids) > 3:
            # matchImagePoints equivalent: board.getChessboardSize() handles the points 
            # internally during calibrateCamera, but we'll collect them manually here
            obj_pts, img_pts = board.matchImagePoints(charuco_corners, charuco_ids)
            
            if img_pts is not None and obj_pts is not None:
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
                all_image_points.append(img_pts)
                all_object_points.append(obj_pts)
                all_images.append(frame)
                image_size = (frame.shape[1], frame.shape[0])
                print(f"Frame captured. Total: {len(all_image_points)}")
            else:
                print("Point matching failed.")

    cap.release()
    cv.destroyAllWindows()

    if len(all_image_points) < 4:
        print("Not enough corners for calibration.")
        return

    # --- Calibration ---
    calibration_flags = 0
    camera_matrix = np.eye(3, 3, dtype=np.float64)
    
    if args.a:
        calibration_flags |= cv.CALIB_FIX_ASPECT_RATIO
        camera_matrix[0, 0] = args.a
    if args.zt:
        calibration_flags |= cv.CALIB_ZERO_TANGENT_DIST
    if args.pc:
        calibration_flags |= cv.CALIB_FIX_PRINCIPAL_POINT

    print("Calibrating... please wait.")
    rep_error, camera_matrix, dist_coeffs, rvecs, tvecs = cv.calibrateCamera(
        all_object_points, all_image_points, image_size, camera_matrix, None, flags=calibration_flags
    )

    # --- Save Results ---
    # Using OpenCV FileStorage to mimic the .yml saving in C++
    fs = cv.FileStorage(args.outfile, cv.FILE_STORAGE_WRITE)
    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.write("camera_matrix", camera_matrix)
    fs.write("distortion_coefficients", dist_coeffs)
    fs.write("avg_reprojection_error", rep_error)
    fs.release()

    print(f"Rep Error: {rep_error}")
    print(f"Calibration saved to {args.outfile}")

    # --- Post-Calibration Debug View ---
    if args.sc:
        for i in range(len(all_images)):
            img_copy = all_images[i].copy()
            cv.aruco.drawDetectedCornersCharuco(img_copy, all_charuco_corners[i], all_charuco_ids[i])
            cv.imshow("Result", img_copy)
            if cv.waitKey(0) == 27:
                break

if __name__ == "__main__":
    main()