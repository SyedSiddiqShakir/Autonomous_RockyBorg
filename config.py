#!/usr/bin/env python3
#configuration file for the line-following robot
#contains parameters used across modules

#GPIO pin configuration
SERVO_PIN = 18  # GPIO18, physical pin 12

#motor control parameters
BASE_SPEED = 0.4
MOTOR_INVERT = -1  # if inversion is needed for a motor rather than changing wires physically @Ravi will still use wires tho, idk why

#PID values, keep tweaking
PID_KP = 45.0
PID_KI = 0.0
PID_KD = 15.0

#camera config
CAMERA_CONFIG = {
    "main": {"size": (2400, 1800)},
    "controls": {
        "AwbEnable": True,
        "AeEnable": True,
        "ExposureTime": 30000,
        "AnalogueGain": 4.0,
        "FrameRate": 30
    }
}

#roi parameters
ROI_WIDTH_RATIO = 0.6
ROI_HEIGHT_RATIO = 0.4
ROI_Y_OFFSET = 0.3

#CLAHE things
CLAHE_CLIP_LIMIT = 3.0
CLAHE_TILE_GRID_SIZE = (8, 8)

#cropping carameters
SIDE_CROP_RATIO = 0.25
VERTICAL_CROP_RATIO = 0.3
FINAL_SIDE_CROP_RATIO = 0.25

#threshold parameters
GAUSSIAN_BLUR_KERNEL = (7, 7)
ADAPTIVE_THRESH_BLOCK_SIZE = 21
ADAPTIVE_THRESH_C = 5

#contour detection parameters
MIN_CONTOUR_AREA = 100
MIN_RED_CONTOUR_AREA = 500
CROSS_JUNCTION_MIN_CONTOURS = 3

#red object detection HSV ranges
RED_HSV_LOWER1 = [0, 120, 70]
RED_HSV_UPPER1 = [10, 255, 255]
RED_HSV_LOWER2 = [160, 120, 70]
RED_HSV_UPPER2 = [180, 255, 255]

#morphological operations, keeping them for now, will decide later
MORPHOLOGY_KERNEL_SIZE = (5, 5)
MORPHOLOGY_ITERATIONS = 1

#servo
SERVO_FREQUENCY = 50  # 50Hz PWM frequency DON'T CHANGE!
SERVO_CENTER_ANGLE = 90
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180
SERVO_MOVE_DELAY = 0.02

#visual overlay
CENTER_LINE_MARGIN = 20