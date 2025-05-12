#!/usr/bin/env python
# coding: utf-8

import cv2
import numpy as np
import time
import sys
sys.path.insert(0, '/home/admin/project/rockyborg/')
from RockyBorg import RockyBorg

def main():
    # Initialize RockyBorg
    RB = RockyBorg()
    RB.Init()
    #RB.SetEpoIgnore(True)
    RB.SetCommsFailsafe(False)
    RB.SetMotorsEnabled(True)

    # Initialize camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Line detection settings
    THRESHOLD = 50
    MIN_LINE_WIDTH = 30
    ROI_HEIGHT_RATIO = 0.4
    BASE_SPEED = 0.4
    MAX_SPEED = 0.4
    TURN_GAIN = 0.8
    MOTOR_INVERT = -1  # Flip motor direction if needed

    try:
        print("Running line follower. Press 'q' to exit.")
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape
            roi = gray[int(h * (1 - ROI_HEIGHT_RATIO)):h, 0:w]

            _, binary = cv2.threshold(roi, THRESHOLD, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            line_found = False
            line_center = 0

            if contours:
                largest = max(contours, key=cv2.contourArea)
                x, y, cw, ch = cv2.boundingRect(largest)

                if cw > MIN_LINE_WIDTH:
                    cx = x + cw // 2
                    cy = y + ch // 2
                    line_center = ((cx / w) * 2) - 1  # Normalize to [-1, 1]
                    line_found = True

                    # Draw visuals
                    cv2.rectangle(frame, (x, int(h * (1 - ROI_HEIGHT_RATIO) + y)),
                                  (x + cw, int(h * (1 - ROI_HEIGHT_RATIO) + y + ch)), (255, 0, 0), 2)
                    cv2.circle(frame, (cx, int(h * (1 - ROI_HEIGHT_RATIO)) + cy), 5, (0, 0, 255), -1)

            if line_found:
                error = line_center
                turn = error * TURN_GAIN
                left_speed = BASE_SPEED + turn
                right_speed = BASE_SPEED - turn
                left_speed = np.clip(left_speed, -MAX_SPEED, MAX_SPEED)
                right_speed = np.clip(right_speed, -MAX_SPEED, MAX_SPEED)

                RB.SetMotor1(left_speed * 1)
                RB.SetMotor2(right_speed * MOTOR_INVERT)
                print(f"\rLine found: Error={error:.2f}", end="")
            else:
                RB.MotorsOff()
                print("\rNo line detected. Stopped.      ", end="")

            cv2.imshow("Camera", frame)
            cv2.imshow("Binary ROI", binary)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        RB.MotorsOff()
        cap.release()
        cv2.destroyAllWindows()
        print("\nShutdown complete.")

if __name__ == "__main__":
    main()
