# Autonomous_RockyBorg
# Branch - CLAUDE

A line-following robot implementation using Raspberry Pi, PiCamera, and servo steering control. The robot can follow black lines, detect cross junctions, and stop when red objects are detected.

## Features

- **PID-controlled servo steering** for precise line following
- **Red object detection** with automatic stop functionality
- **Cross junction detection** with straight-ahead navigation
- **CLAHE image enhancement** for better line visibility
- **Adaptive thresholding** for robust line detection
- **Real-time debug visualization** with multiple camera views

## Hardware Requirements

- Raspberry Pi 3B+
- Pi Camera
- Servo motor (for steering)
- RockyBorg motor controller board
- DC motors for movement

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
