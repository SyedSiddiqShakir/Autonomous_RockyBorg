# Autonomous Line Following RockyBorg

This project is a Raspberry Pi–based autonomous robot built using the **RockyBorg motor controller**, equipped with:
- Front **servo steering**
- Rear **DC motors** x2
- **Pi Camera** for OpenCV + PID lane tracking

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
Autonomous_RockyBorg/
├── main.py              # Main control script
├── vision.py            # OpenCV lane tracking
├── motors.py            # Motor control functions
├── gui_data_receiver.py # Pulls motor data
├── gui_monitor.py       # Live GUI dashboard (optional)
├── start_robot.sh       # Run entrypoint for Pi
├── test_drive.py        # Manual motor + servo testing
├── .gitignore
├── README.md

## Acknowledgements
PiBorg for RockyBorg library (https://github.com/piborg/RockyBorg)
OpenCV tutorials: TechWithTim
Raspberry Pi community
