#!/usr/bin/env python3
"""
Red ball tracker for Raspberry Pi -> STM32G431 robot controller.

Captures frames with picamera2, finds the largest red blob with OpenCV,
and sends single-character drive commands over UART matching the
protocol implemented in uart_control.c on the STM32:

    'F' = forward, 'B' = backward, 'L' = turn left, 'R' = turn right,
    'S' = stop

IMPORTANT: the STM32 firmware has a 500ms UART watchdog (see UART_Update()
in uart_control.c) - if it doesn't receive a byte within that window it
stops the motors. A background thread here resends the current command
every COMMAND_RESEND_PERIOD_S on its own schedule, independent of camera
frame rate, so a slow or dropped frame can never cause an unexpected stop.

Setup on the Pi (one-time):
    sudo raspi-config
      -> Interface Options -> Serial Port
         "login shell over serial?" -> No
         "serial port hardware enabled?" -> Yes
    (if your Pi model routes UART through Bluetooth, e.g. Pi 3/4/Zero W,
     also add `dtoverlay=disable-bt` to /boot/firmware/config.txt and
     `sudo systemctl disable hciuart`, then reboot)

    pip install picamera2 opencv-python pyserial numpy

Wiring: Pi GPIO14 (TX) -> STM32 PA3 (RX), Pi GPIO15 (RX) -> STM32 PA2 (TX),
plus a common ground. Both sides are 3.3V logic, so no level shifter needed.
"""

import sys
import time
import threading

import serial
import cv2
import numpy as np
from picamera2 import Picamera2

# ---------------------------------------------------------------------------
# Configuration - tune these against your camera position and lighting
# ---------------------------------------------------------------------------

SERIAL_PORT = "/dev/serial0"     # UART pins on the Pi header (GPIO14/15)
SERIAL_BAUD = 115200             # must match huart2.Init.BaudRate in main.c

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# HSV thresholds for red - red wraps around hue 0/180 so two ranges are
# combined into one mask. Widen/narrow the S and V ranges if the ball
# isn't detecting reliably under your lighting.
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])

MIN_BALL_RADIUS_PX = 8       # ignore blobs smaller than this (noise)
CENTER_DEADZONE_PX = 40      # +/- this many px from center = "centered"

# Ball apparent radius used as a rough distance proxy - a bigger radius
# means the ball is closer to the camera. Measure your actual ball at a
# few known distances and adjust these two numbers accordingly.
RADIUS_TOO_FAR_PX = 30       # smaller than this -> drive forward
RADIUS_TOO_CLOSE_PX = 70     # bigger than this -> back up

# Must stay comfortably under the STM32's 500ms UART watchdog timeout.
COMMAND_RESEND_PERIOD_S = 0.15
NO_BALL_STOP_TIMEOUT_S = 1.0     # stop only after losing the ball this long

SHOW_DEBUG_WINDOW = False    # True if running with a display attached


# ---------------------------------------------------------------------------


class SerialCommander:
    """Owns the serial link to the STM32 and guarantees it gets a fresh
    byte at least every COMMAND_RESEND_PERIOD_S, on its own thread and
    schedule. This decouples the STM32's UART watchdog from the camera/
    vision loop's speed - if a frame takes a while to process, the
    keepalive still goes out on time and the robot won't unexpectedly
    stop (or, worse, keep executing a stale command past when it should
    have been updated)."""

    def __init__(self, port, baud, resend_period_s):
        self._ser = serial.Serial(port, baud, timeout=0.1)
        self._resend_period_s = resend_period_s
        self._lock = threading.Lock()
        self._command = 'S'
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_command(self, command):
        with self._lock:
            self._command = command

    def _run(self):
        while not self._stop_event.is_set():
            with self._lock:
                command = self._command
            try:
                self._ser.write(command.encode('ascii'))
            except serial.SerialException as exc:
                print(f"Serial write failed: {exc}", file=sys.stderr)
            time.sleep(self._resend_period_s)

    def close(self):
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        try:
            self._ser.write(b'S')
            time.sleep(0.05)
        except serial.SerialException:
            pass
        self._ser.close()


class BallTracker:
    def __init__(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1.0)  # let auto-exposure/white-balance settle

    def find_ball(self, frame_bgr):
        """Return (cx, cy, radius) of the largest red blob, or None."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
        mask2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Clean up noise: erode away small specks, then dilate back up
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest)

        if radius < MIN_BALL_RADIUS_PX:
            return None

        return (int(x), int(y), int(radius))

    def capture_frame(self):
        rgb = self.picam2.capture_array()  # picamera2 gives RGB
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)  # OpenCV wants BGR

    def close(self):
        self.picam2.stop()


def decide_command(ball, frame_width):
    """Map a detected ball position/size to a single drive command char."""
    if ball is None:
        return None

    cx, _cy, radius = ball
    frame_center_x = frame_width // 2
    offset_x = cx - frame_center_x

    # Turning takes priority over forward/back, so the robot centers on
    # the ball before it starts closing distance.
    if offset_x < -CENTER_DEADZONE_PX:
        return 'L'
    if offset_x > CENTER_DEADZONE_PX:
        return 'R'

    if radius < RADIUS_TOO_FAR_PX:
        return 'F'
    if radius > RADIUS_TOO_CLOSE_PX:
        return 'B'

    return 'S'  # centered and at a good distance


def main():
    try:
        commander = SerialCommander(SERIAL_PORT, SERIAL_BAUD, COMMAND_RESEND_PERIOD_S)
    except serial.SerialException as exc:
        print(f"Failed to open serial port {SERIAL_PORT}: {exc}", file=sys.stderr)
        print(
            "Check that UART is enabled (raspi-config -> Interface Options "
            "-> Serial Port) and that nothing else (e.g. a login shell) "
            "is using it.",
            file=sys.stderr,
        )
        return 1

    try:
        tracker = BallTracker()
    except Exception as exc:  # picamera2 raises various error types depending on cause
        print(f"Failed to start camera: {exc}", file=sys.stderr)
        print(
            "Check that the camera is connected and enabled "
            "(raspi-config -> Interface Options -> Camera).",
            file=sys.stderr,
        )
        commander.close()
        return 1

    last_command = 'S'
    last_seen_time = time.monotonic()

    print("Red ball tracker running. Ctrl+C to stop.")

    try:
        while True:
            frame = tracker.capture_frame()
            ball = tracker.find_ball(frame)
            now = time.monotonic()

            if ball is not None:
                last_seen_time = now
                command = decide_command(ball, FRAME_WIDTH)
            else:
                # Ball not visible in this particular frame. Only stop
                # once it's actually been missing for a while, so one
                # dropped/blurry frame doesn't jerk the robot to a halt.
                if (now - last_seen_time) > NO_BALL_STOP_TIMEOUT_S:
                    command = 'S'
                else:
                    command = last_command

            if command != last_command:
                print(f"-> {command}")
            last_command = command
            commander.set_command(command)

            if SHOW_DEBUG_WINDOW:
                if ball is not None:
                    cx, cy, radius = ball
                    cv2.circle(frame, (cx, cy), radius, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
                cv2.putText(
                    frame, f"cmd: {last_command}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2
                )
                cv2.imshow("Tracker", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        commander.set_command('S')
        time.sleep(COMMAND_RESEND_PERIOD_S * 2)  # let the keepalive thread flush a stop
        commander.close()
        tracker.close()
        if SHOW_DEBUG_WINDOW:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
