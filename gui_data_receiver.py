#!/usr/bin/env python3
import time
from RockyBorg import RockyBorg

#initialization

RB = RockyBorg()
RB.Init()
RB.SetEpoIgnore(True)  # Ignore Emergency Power Off
RB.SetCommsFailsafe(False)  # Disable auto shutdown
RB.SetMotorsEnabled(True)  # Enable motor outputs

#main control functions, can be used as a standard wrapper

def move_forward(speed=0.6, duration=2):
    print(f"Moving forward at {speed}")
    RB.SetMotors(speed)
    time.sleep(duration)
    RB.SetMotors(0)

def move_backward(speed=0.6, duration=2):
    print(f"Moving backward at {speed}")
    RB.SetMotors(-speed)
    time.sleep(duration)
    RB.SetMotors(0)

def turn_left(angle=0.5, duration=1):
    print(f"Turning left with servo position: {-angle}")
    RB.SetServoPosition(-angle)
    time.sleep(duration)
    RB.SetServoPosition(0)

def turn_right(angle=0.5, duration=1):
    print(f"Turning right with servo position: {angle}")
    RB.SetServoPosition(angle)
    time.sleep(duration)
    RB.SetServoPosition(0)

def stop_all():
    RB.SetMotors(0)
    RB.SetServoPosition(0)
    print("Stopped all motors and servo")

#printing status of motors 

def print_current_status():
    m1 = RB.GetMotor1()
    m2 = RB.GetMotor2()
    servo = RB.GetServoPosition()
    print(f"[Motor 1] Speed: {m1:.2f}")
    print(f"[Motor 2] Speed: {m2:.2f}")
    print(f"[Servo ] Angle: {servo:.2f}")

#testing
if __name__ == "__main__":
    try:
        move_forward()
        print_current_status()

        turn_left()
        print_current_status()

        move_backward()
        print_current_status()

        turn_right()
        print_current_status()

        stop_all()

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        stop_all()
