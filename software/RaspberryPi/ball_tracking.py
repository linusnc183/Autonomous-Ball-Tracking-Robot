#!/usr/bin/env python3
"""
Red ball tracker -> STM32G431 robot controller over UART.
Commands match uart_control.c: F/B/L/R/S.

Setup:
    raspi-config -> Interface Options -> Serial Port (enable, disable login shell)
    pip install picamera2 opencv-python pyserial numpy

Wiring: Pi TX(GPIO14) -> STM32 PA3, Pi RX(GPIO15) -> STM32 PA2, common GND.
"""

import sys
import time
import threading

import serial
import cv2
import numpy as np
from picamera2 import Picamera2

SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 115200

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# red wraps hue 0/180, so two ranges
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])

MIN_BALL_RADIUS_PX = 8
CENTER_DEADZONE_PX = 40

RADIUS_TOO_FAR_PX = 30
RADIUS_TOO_CLOSE_PX = 70

COMMAND_RESEND_PERIOD_S = 0.15   # stay under the STM32's 500ms watchdog
NO_BALL_STOP_TIMEOUT_S = 1.0

SHOW_DEBUG_WINDOW = False


class SerialCommander:
    """Resends the current command on its own thread so the STM32's
    UART watchdog never trips due to camera frame time."""

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
        time.sleep(1.0)  # let AE/AWB settle

    def find_ball(self, frame_bgr):
        """Return (cx, cy, radius) of the largest red blob, or None."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
        mask2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(largest)
        if radius < MIN_BALL_RADIUS_PX:
            return None

        return (int(x), int(y), int(radius))

    def capture_frame(self):
        rgb = self.picam2.capture_array()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def close(self):
        self.picam2.stop()


def decide_command(ball, frame_width):
    if ball is None:
        return None

    cx, _cy, radius = ball
    offset_x = cx - frame_width // 2

    if offset_x < -CENTER_DEADZONE_PX:
        return 'L'
    if offset_x > CENTER_DEADZONE_PX:
        return 'R'
    if radius < RADIUS_TOO_FAR_PX:
        return 'F'
    if radius > RADIUS_TOO_CLOSE_PX:
        return 'B'
    return 'S'


def main():
    try:
        commander = SerialCommander(SERIAL_PORT, SERIAL_BAUD, COMMAND_RESEND_PERIOD_S)
    except serial.SerialException as exc:
        print(f"Failed to open serial port {SERIAL_PORT}: {exc}", file=sys.stderr)
        return 1

    try:
        tracker = BallTracker()
    except Exception as exc:
        print(f"Failed to start camera: {exc}", file=sys.stderr)
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
            elif (now - last_seen_time) > NO_BALL_STOP_TIMEOUT_S:
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
                cv2.putText(frame, f"cmd: {last_command}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow("Tracker", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        commander.set_command('S')
        time.sleep(COMMAND_RESEND_PERIOD_S * 2)
        commander.close()
        tracker.close()
        if SHOW_DEBUG_WINDOW:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
