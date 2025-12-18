# 2-LED Midpoint Tracker — 2-axis tracking + offset calibration
import time
import cv2
import numpy as np
from picamera2 import Picamera2

# GPIO (real on Pi, mock elsewhere so you can at least run the vision loop)
try:
    import RPi.GPIO as GPIO
    print("RPi.GPIO library loaded successfully.")
except (ImportError, RuntimeError):
    print("--- WARNING: RPi.GPIO library not found. Using MOCK GPIO. ---")
    GPIO = type(
        "MockGPIO",
        (),
        {"__getattr__": lambda self, name: print(f"[MOCK] Ignored GPIO call: {name}")},
    )()

# Pins (BCM)
STEP_PIN_X = 4    # PUL+ on X-driver (Vertical/Tilt)
DIR_PIN_X  = 3    # DIR+ on X-driver
STEP_PIN_Y = 11   # PUL+ on Y-driver (Horizontal/Pan)
DIR_PIN_Y  = 9    # DIR+ on Y-driver

# Motor direction flips (set based on your wiring/mechanics)
INVERT_Y_MOTOR_FOR_PAN = True
INVERT_X_MOTOR_FOR_TILT = False

# Motion tuning
STEP_DELAY = 0.0001
MOTOR_STEP_SIZE = 10     # auto tracking step size
MANUAL_STEP_SIZE = 5     # manual arrow step size

# Camera setup
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(0.5)

# Try to lock exposure/gain so thresholding stays stable
try:
    picam2.set_controls(
        {"AeEnable": False, "AwbEnable": False, "ExposureTime": 4000, "AnalogueGain": 1.5}
    )
    print("Locked camera exposure and gain.")
except Exception as e:
    print(f"Could not set camera controls: {e}")

W, H = 640, 480
CENTER_X, CENTER_Y = W // 2, H // 2

# Tracking parameters
SEND_PERIOD = 0.1
DEAD_ZONE_PIXELS = 10
MIN_BRIGHTNESS_THRESHOLD = 150
MIN_CONTOUR_AREA = 25


def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([STEP_PIN_X, DIR_PIN_X, STEP_PIN_Y, DIR_PIN_Y], GPIO.OUT)
    print("GPIO pins configured.")


def move_xy(sx, sy):
    # Set direction based on sign, then issue step pulses in a simple Bresenham-ish mix
    GPIO.output(DIR_PIN_X, GPIO.HIGH if sx > 0 else GPIO.LOW)
    GPIO.output(DIR_PIN_Y, GPIO.HIGH if sy > 0 else GPIO.LOW)

    ax, ay = abs(sx), abs(sy)
    max_steps = max(ax, ay)
    if max_steps == 0:
        return

    err_x = max_steps // 2
    err_y = max_steps // 2

    for _ in range(max_steps):
        err_x -= ax
        if err_x < 0:
            err_x += max_steps
            GPIO.output(STEP_PIN_X, GPIO.HIGH)

        err_y -= ay
        if err_y < 0:
            err_y += max_steps
            GPIO.output(STEP_PIN_Y, GPIO.HIGH)

        time.sleep(0.000001)
        GPIO.output(STEP_PIN_X, GPIO.LOW)
        GPIO.output(STEP_PIN_Y, GPIO.LOW)
        time.sleep(STEP_DELAY)


print("Starting Tracker.")
print(" CONTROLS:")
print("  [A] - Auto Mode (Follow LEDs)")
print("  [M] - Manual Mode (Stop tracking logic)")
print("  [S] - SET OFFSET (Records current LED pos as new target)")
print("  [R] - RESET OFFSET (Back to screen center)")
print("  [Arr] - Move motors manually")
print("  [Q] - Quit")

setup_gpio()
last_send_t = 0.0

# State
is_auto_mode = True
target_lock_x = CENTER_X  # where you want the LED midpoint to sit
target_lock_y = CENTER_Y

try:
    while True:
        frame_bgr = picam2.capture_array()
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        # Threshold + contours (simple bright-spot detection)
        _, threshold_img = cv2.threshold(
            gray, MIN_BRIGHTNESS_THRESHOLD, 255, cv2.THRESH_BINARY
        )
        contours, _ = cv2.findContours(
            threshold_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > MIN_CONTOUR_AREA:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    valid_contours.append(
                        {"contour": cnt, "area": area, "center": (cX, cY)}
                    )

        valid_contours.sort(key=lambda x: x["area"], reverse=True)

        # Default midpoint (safe fallback)
        current_led_mid_x, current_led_mid_y = CENTER_X, CENTER_Y
        led1, led2 = None, None
        target_found = False

        # Use the 2 largest blobs as the LEDs
        if len(valid_contours) >= 2:
            led1 = valid_contours[0]
            led2 = valid_contours[1]
            current_led_mid_x = (led1["center"][0] + led2["center"][0]) // 2
            current_led_mid_y = (led1["center"][1] + led2["center"][1]) // 2
            target_found = True

        # Keyboard
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("a"):
            is_auto_mode = True
            print(">> Switched to AUTO MODE")
        elif key == ord("m"):
            is_auto_mode = False
            print(">> Switched to MANUAL MODE")
        elif key == ord("r"):
            target_lock_x, target_lock_y = CENTER_X, CENTER_Y
            print(">> OFFSET RESET to Center")
        elif key == ord("s"):
            # Set offset: lock target to current LED midpoint
            if target_found:
                target_lock_x = current_led_mid_x
                target_lock_y = current_led_mid_y
                print(f">> OFFSET SET. Locking LEDs to ({target_lock_x}, {target_lock_y})")
            else:
                print(">> ERROR: Cannot set offset, LEDs not found!")

        # Decide steps
        step_x, step_y = 0, 0
        status_text = ""
        status_color = (0, 255, 0)

        if is_auto_mode:
            if target_found:
                # Error is relative to target_lock (not necessarily screen center)
                offx = current_led_mid_x - target_lock_x
                offy = current_led_mid_y - target_lock_y

                if abs(offx) > DEAD_ZONE_PIXELS:
                    step_y = -MOTOR_STEP_SIZE if offx > 0 else MOTOR_STEP_SIZE
                if abs(offy) > DEAD_ZONE_PIXELS:
                    step_x = -MOTOR_STEP_SIZE if offy > 0 else MOTOR_STEP_SIZE

                status_text = "AUTO: TRACKING"
            else:
                status_text = "AUTO: SEARCHING"
                status_color = (0, 165, 255)
        else:
            status_text = "MANUAL MODE"
            status_color = (0, 0, 255)

            # Arrow keys: Up(82), Down(84), Left(81), Right(83)
            if key == 82:
                step_x = MANUAL_STEP_SIZE
            elif key == 84:
                step_x = -MANUAL_STEP_SIZE
            elif key == 81:
                step_y = MANUAL_STEP_SIZE
            elif key == 83:
                step_y = -MANUAL_STEP_SIZE

        # Apply inversion flags
        if INVERT_X_MOTOR_FOR_TILT:
            step_x = -step_x
        if INVERT_Y_MOTOR_FOR_PAN:
            step_y = -step_y

        # Move (rate-limited in auto, immediate in manual)
        now = time.time()
        if (not is_auto_mode) and (step_x != 0 or step_y != 0):
            move_xy(step_x, step_y)
        elif is_auto_mode and (now - last_send_t) >= SEND_PERIOD:
            if step_x != 0 or step_y != 0:
                move_xy(step_x, step_y)
            last_send_t = now

        # Overlay UI
        # Purple cross = lock target (where you want midpoint to be)
        cv2.drawMarker(
            frame_bgr,
            (target_lock_x, target_lock_y),
            (255, 0, 255),
            cv2.MARKER_CROSS,
            30,
            2,
        )

        if target_found and led1 and led2:
            # LEDs = blue contours
            cv2.drawContours(
                frame_bgr, [led1["contour"], led2["contour"]], -1, (255, 0, 0), 2
            )
            # LED-to-LED line = yellow
            cv2.line(frame_bgr, led1["center"], led2["center"], (0, 255, 255), 2)
            # Current midpoint = green dot
            cv2.circle(
                frame_bgr, (current_led_mid_x, current_led_mid_y), 10, (0, 255, 0), -1
            )

            # Error line (midpoint -> lock target) in auto
            if is_auto_mode:
                cv2.line(
                    frame_bgr,
                    (current_led_mid_x, current_led_mid_y),
                    (target_lock_x, target_lock_y),
                    (255, 255, 255),
                    1,
                )

        cv2.putText(
            frame_bgr,
            status_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            status_color,
            2,
        )
        cv2.putText(
            frame_bgr,
            "[S] Set Offset  [R] Reset",
            (10, H - 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (200, 200, 200),
            1,
        )
        cv2.putText(
            frame_bgr,
            "[A] Auto  [M] Manual",
            (10, H - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (200, 200, 200),
            1,
        )

        cv2.imshow("2-LED Tracker", frame_bgr)

finally:
    cv2.destroyAllWindows()
    picam2.stop()
    GPIO.cleanup()
    print("Camera stopped and GPIO cleaned up.")
