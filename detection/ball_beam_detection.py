import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import numpy as np
import cv2 as cv
import undistort as ud
from plotter import initialize_plotter

send_data, plot_data, read_latest_serial, update_ui, consume_key = initialize_plotter()

# ── Calibration state ────────────────────────────────────────────────────────
MIN_SAMPLES      = 5      # minimum pairs before a fit is computed
calib_mode       = False  # True while collecting samples
calib_samples    = []     # list of (cv_px, tof_mm) tuples
calib_fit        = None   # (slope, intercept) after fitting, or None
calibrated       = False   # Is the data done being calibrated?
SAMPLE_COOLDOWN  = 15     # frames to wait between auto-samples in calib mode
sample_cooldown  = 0

def fit_calibration(samples):
    """Least-squares linear fit: tof = slope * cv_px + intercept."""
    xs = np.array([s[0] for s in samples], dtype=np.float64)
    ys = np.array([s[1] for s in samples], dtype=np.float64)
    slope, intercept = np.polyfit(xs, ys, 1)
    return slope, intercept

def apply_calibration(cv_px, fit):
    """Convert raw CV pixel distance to calibrated real-world distance."""
    if fit is None:
        return cv_px                        # pass-through before calibration
    slope, intercept = fit
    return slope * cv_px + intercept
# ─────────────────────────────────────────────────────────────────────────────

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
    t  = np.dot(AP, AB) / np.dot(AB, AB)
    return A + t * AB

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FPS, 30.0)
# cap.set(cv.CAP_PROP_FRAME_WIDTH,  1280)
# cap.set(cv.CAP_PROP_FRAME_HEIGHT,  720)
cap.set(cv.CAP_PROP_FRAME_WIDTH,  1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,  1080)

cv.namedWindow('mask', cv.WINDOW_NORMAL)
cv.resizeWindow('mask', 640, 400)
cv.createTrackbar('hue_lower', 'mask',  76, 179, nothing)
cv.createTrackbar('hue_upper', 'mask',  97, 179, nothing)
cv.createTrackbar('min_sat',   'mask', 110, 255, nothing)
cv.createTrackbar('min_value', 'mask', 100, 255, nothing)

lower_ball = np.array([ 5, 140, 140])
upper_ball = np.array([20, 255, 255])

crosshair_length = 100
dist_raw = 0.0
dist_cal = 0.0

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame")
        break

    # ── HSV trackbars ────────────────────────────────────────────────────────
    lower_hue = cv.getTrackbarPos('hue_lower', 'mask')
    upper_hue = cv.getTrackbarPos('hue_upper', 'mask')
    min_sat   = cv.getTrackbarPos('min_sat',   'mask')
    min_value = cv.getTrackbarPos('min_value', 'mask')
    lower_beam = np.array([lower_hue, min_sat,   min_value])
    upper_beam = np.array([upper_hue,      255,        255])

    frame      = ud.undistort(frame)
    blur_frame = cv.GaussianBlur(frame, (5, 5), 0)
    hsv        = cv.cvtColor(blur_frame, cv.COLOR_BGR2HSV)

    # ── Ball detection ───────────────────────────────────────────────────────
    ball_mask = cv.inRange(hsv, lower_ball, upper_ball)
    cx, cy    = find_moment(ball_mask)
    if cx is not None:
        cv.putText(frame, f"X: {cx:.02f} Y: {cy:.02f}",
                   (int(cx) + 10, int(cy) - 10),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cx, cy = round(cx), round(cy)
        cv.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
        cv.line(frame, (cx - crosshair_length, cy), (cx + crosshair_length, cy), (0, 0, 0), 2)
        cv.line(frame, (cx, cy - crosshair_length), (cx, cy + crosshair_length), (0, 0, 0), 2)

    # ── Beam detection & distance ────────────────────────────────────────────
    beam_mask = cv.inRange(hsv, lower_beam, upper_beam)
    got_dist  = False

    if beam_mask is not None:
        beam_mask_left, beam_mask_right = np.array_split(beam_mask, 2, 1)
        xb0, yb0 = find_moment(beam_mask_left)
        xb1, yb1 = find_moment(beam_mask_right)

        if xb0 is not None:
            cv.circle(frame, (round(xb0), round(yb0)), 8, (0, 0, 255), -1)
        if xb1 is not None:
            xb1 += beam_mask_left.shape[1]
            cv.circle(frame, (round(xb1), round(yb1)), 8, (0, 0, 255), -1)

        if xb0 is not None and xb1 is not None and cx is not None:
            cv.line(frame, (round(xb0), round(yb0)), (round(xb1), round(yb1)), (255, 0, 0), 2)
            A = np.array([xb0, yb0], dtype=np.float32)
            B = np.array([xb1, yb1], dtype=np.float32)
            P = np.array([cx,  cy],  dtype=np.float32)
            Q = calc_proj(A, B, P)

            dist_raw = float(np.linalg.norm(Q - A))
            dist_cal = apply_calibration(dist_raw, calib_fit)
            got_dist = True

            cv.circle(frame, tuple(Q.astype(int)), 8, (0, 0, 255), -1)
            cv.line(frame, tuple(P.astype(int)), tuple(Q.astype(int)), (0, 255, 0), 2)

    # ── Calibration: auto-sample in calib mode ───────────────────────────────
    if calib_mode and got_dist and sample_cooldown == 0:
        tof_val = read_latest_serial()           # your serial ToF reading
        if tof_val is not None:
            calib_samples.append((dist_raw, float(tof_val)))
            if len(calib_samples) >= MIN_SAMPLES:
                calib_fit = fit_calibration(calib_samples)
            sample_cooldown = SAMPLE_COOLDOWN

    if sample_cooldown > 0:
        sample_cooldown -= 1

    # ── HUD overlay ─────────────────────────────────────────────────────────
    hud_y = 30
    hud_color = (0, 255, 255)

    if calib_mode:
        status = f"CALIBRATING  samples={len(calib_samples)}"
        if calib_fit:
            status += f"  fit=({calib_fit[0]:.4f}x + {calib_fit[1]:.2f})"
        cv.putText(frame, status, (10, hud_y),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 2)
        hud_y += 25

    if calib_fit:
        cv.putText(frame, f"cal dist: {dist_cal:.1f}  raw px: {dist_raw:.1f}",
                   (10, hud_y), cv.FONT_HERSHEY_SIMPLEX, 0.6, hud_color, 2)
        if not calib_mode:
            calibrated = True
    else:
        cv.putText(frame, f"raw px: {dist_raw:.1f}  (no calibration)",
                   (10, hud_y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 2)

    cv.putText(frame, "[C] calib  [R] reset  [ESC] quit",
               (10, frame.shape[0] - 10), cv.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    # ── Send downstream ──────────────────────────────────────────────────────
    if got_dist:
        plot_data(dist_cal)
        update_ui()
        if calibrated:
            send_data(dist_cal)

    cv.imshow('display', frame)
    cv.imshow('mask', beam_mask)

    # ── Key handling ─────────────────────────────────────────────────────────
    cv.waitKey(1)
    if consume_key('esc'):
        break
    if consume_key('c'):
        calib_mode = not calib_mode
        print(f"Calibration mode {'ON' if calib_mode else 'OFF'}")
    if consume_key('r'):
        calib_samples.clear()
        calib_fit    = None
        calib_mode   = False
        calibrated   = False
        sample_cooldown = 0
        print("Calibration reset")

cap.release()
cv.destroyAllWindows()