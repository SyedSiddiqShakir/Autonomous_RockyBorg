from picamera2 import Picamera2
from time import sleep
import cv2

picam2 = None


def init_camera():
    global picam2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={'size': (800, 600)}, controls={
        "AwbEnable": True,
        "AeEnable": True,
        "ExposureTime": 30000,
        "AnalogueGain": 4.0,
        "FrameRate": 30
    })
    picam2.configure(config)


def start_camera():
    global picam2
    if picam2 is None:
        picam2.start()
    sleep(2)


def stop_camera():
    global picam2
    if picam2 is not None:
        picam2.stop()


def get_frame():
    global picam2
    if picam2 is None:
        frame = picam2.capture_array()
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return None
