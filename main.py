#!/usr/bin/env python

from picamera2 import Picamera2
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
    RB.SetCommsFailsafe(False)
    RB.SetMotorsEnabled(True)

    # Initialize PiCamera2 with low-light optimization
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (800, 600)},
        controls={
            "AwbEnable": True,
            "AeEnable": True,
            "ExposureTime": 30000,
            "AnalogueGain": 4.0,
            "FrameRate": 30
        }
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)

    # Control parameters
    BASE_SPEED = 0.4
    TURN_GAIN = 0.5
    MOTOR_INVERT = -1

    def clamp_power(value):
        return max(-1.0, min(1.0, value))

    try:
        print("Line following with red object detection. Press 'q' to exit.")
        while True:
            # Capture and process frame
            frame = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # ROI configuration
            h, w = frame_bgr.shape[:2]
            roi_w = int(w * 0.6)
            roi_h = int(h * 0.4)
            roi_x = int((w - roi_w) / 2)
            roi_y = int(h * 0.3)
            cropped = frame_bgr[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
            
            # Image enhancement
            lab = cv2.cvtColor(cropped, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
            limg = clahe.apply(l)
            enhanced = cv2.merge([limg, a, b])
            enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
            
            # Multi-stage cropping
            side_crop = int(cropped.shape[1] * 0.25)
            narrow_frame = enhanced[:, side_crop:-side_crop]
            temp_frame = narrow_frame[narrow_frame.shape[0]//2:, :]
            final_frame = temp_frame[int(temp_frame.shape[0]*0.3):, :]
            final_side_crop = int(final_frame.shape[1] * 0.25)
            extreme_narrow_frame = final_frame[:, final_side_crop:-final_side_crop]

            # Line detection
            gray = cv2.cvtColor(extreme_narrow_frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (7,7), 0)
            binary = cv2.adaptiveThreshold(blurred, 255, 
                                         cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv2.THRESH_BINARY_INV, 21, 5)

            # Line tracking logic
            line_found = False
            line_center = 0
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 100]
                if valid_contours:
                    largest = max(valid_contours, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(largest)
                    line_found = True
                    line_center = ((x + w/2) / binary.shape[1]) * 2 - 1
                    cv2.rectangle(extreme_narrow_frame, (x,y), (x+w,y+h), (0,255,0), 2)

            # Red object detection
            red_detected = False  # Proper initialization
            hsv = cv2.cvtColor(extreme_narrow_frame, cv2.COLOR_BGR2HSV)
            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 120, 70])
            upper_red2 = np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            
            # Noise reduction
            kernel = np.ones((5,5), np.uint8)
            red_mask = cv2.erode(red_mask, kernel, iterations=1)
            red_mask = cv2.dilate(red_mask, kernel, iterations=1)
            
            # Contour analysis
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in red_contours:
                if cv2.contourArea(cnt) > 500:
                    red_detected = True
                    cv2.drawContours(extreme_narrow_frame, [cnt], -1, (0,0,255), 2)
                    break

            # Motor control logic
            if red_detected:
                RB.MotorsOff()
                status = "RED DETECTED - STOPPED"
            elif line_found:
                turn = line_center * TURN_GAIN
                left = clamp_power(BASE_SPEED + turn)
                right = clamp_power(BASE_SPEED - turn)
                RB.SetMotor1(left)
                RB.SetMotor2(right * MOTOR_INVERT)
                status = f"FOLLOWING: {line_center:.2f}"
            else:
                RB.MotorsOff()
                status = "NO LINE"

            # Display outputs
            cv2.imshow("Processing View", extreme_narrow_frame)
            cv2.imshow("Binary", binary)
            cv2.imshow("Red Mask", red_mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        RB.MotorsOff()
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
