#!/usr/bin/env python3
#keep all the servo stuff here and the pid as well

import time
import RPi.GPIO as GPIO
from config import *

class ServoController:
    def __init__(self):
        #sets up GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQUENCY)
        self.servo_pwm.start(0)
        
        #PID variables
        self.integral = 0.0
        self.previous_error = 0.0
        
        #center servo initially
        self.set_angle(SERVO_CENTER_ANGLE)
    
    def set_angle(self, angle):
        angle = self.clamp(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)
        
        #map angle (0-180) to duty cycle (2.5 - 12.5)
        duty = 2.5 + (angle / 180.0) * 10.0
        self.servo_pwm.ChangeDutyCycle(duty)
        time.sleep(SERVO_MOVE_DELAY)
        self.servo_pwm.ChangeDutyCycle(0)  #stops signal to reduce jitter
    
    def calculate_pid_steering(self, line_center):
        #convert line center (-1 to 1) to error. Is it called normalization? idk
        error = -line_center  #invert if needed to match steering direction
        
        #PID calculations
        self.integral += error
        derivative = error - self.previous_error
        
        pid_output = PID_KP * error + PID_KI * self.integral + PID_KD * derivative
        self.previous_error = error
        
        #convert PID output to servo angle
        angle = SERVO_CENTER_ANGLE + pid_output
        return self.clamp(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)
    
    def reset_pid(self):
        self.integral = 0.0
        self.previous_error = 0.0
    
    def center_servo(self):
        self.set_angle(SERVO_CENTER_ANGLE)
    
    @staticmethod #probably works
    def clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))
    
    def cleanup(self):
        self.servo_pwm.stop()
        GPIO.cleanup()