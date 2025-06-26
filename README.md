
# Autonomous_RockyBorg

## Features
- **PID-controlled servo steering** for precise line following
- **Red object detection** with automatic stop functionality
- **Cross junction detection** with straight-ahead navigation
- **CLAHE image enhancement** for better line visibility
- **Adaptive thresholding** for robust line detection
- **Real-time debug visualization** with multiple camera views

## Hardware Used

| Component              | Description                         |
|------------------------|-------------------------------------|
| Raspberry Pi 3B+       | Main controller                     |
| RockyBorg PCB          | Motor + servo control via I2C       |
| 2x DC Motors           | Rear drive motors                   |
| 1x Servo Motor         | Front steering                      |
| Pi Camera              | Lane detection using OpenCV         |
| Power Supply           | 8x AA or 2x 18650 batteries         |

## Software Features

- Manual control via RockyBorg GUI or joystick
- Line Following using camera and OpenCV
- Remote access via `connect.raspberrypi.com`

## Project Structure

```
servo-pid-red-stop/
├── README.md
├── main.py                 # Main control loop and robot coordination
├── config.py              # Configuration parameters and constants
├── servo_controller.py     # Servo PWM control and PID steering logic
├── vision_processor.py     # Camera handling and computer vision processing
└── motor_controller.py     # RockyBorg motor control interface
```

## Acknowledgements
PiBorg for RockyBorg library (https://github.com/piborg/RockyBorg)
OpenCV tutorials: TechWithTim
Raspberry Pi community
VisionRace Masters (https://github.com/CRM-UAM/VisionRace/tree/master)
Vision race drive test (https://github.com/CRM-UAM/VisionRace/tree/drive_test)
Robot Positioning (x-IMU) (https://github.com/xioTechnologies/Gait-Tracking-With-x-IMU/tree/master)
Line Following Robot OpenCV (https://www.youtube.com/watch?v=ZC4VUt1I5FI)
Line Tracking (https://github.com/fustyles/Arduino/blob/master/ESP32-CAM_Car/ESP32-CAM_CAR_TrackColorline_Tracking.js/ESP32-CAM_CAR_TrackColorline_Tracking.js.ino)