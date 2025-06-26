#!/usr/bin/env python3
#uses I2C base commands from rockyborg library 

import sys
sys.path.insert(0, '/home/admin/project/rockyborg/')
from RockyBorg import RockyBorg
from config import *

class MotorController:
    def __init__(self):
        self.RB = RockyBorg()
        self.setup_motors()
    
    def setup_motors(self):
        #starts motors
        self.RB.Init()
        self.RB.SetCommsFailsafe(False)
        self.RB.SetMotorsEnabled(True)
    
    def move_forward(self, speed=None):
        if speed is None:
            speed = BASE_SPEED #idk how efficient the config module is, lets keep for now
        self.RB.SetMotor1(speed)
        self.RB.SetMotor2(speed * MOTOR_INVERT)
    
    def stop_motors(self):
        self.RB.MotorsOff()
    
    def move_with_custom_speeds(self, left_speed, right_speed):
        self.RB.SetMotor1(left_speed)
        self.RB.SetMotor2(right_speed * MOTOR_INVERT)
    
    def get_motor_status(self):
        #this could be extended to return actual motor status if RockyBorg supports it
        return "Motors Active"
    
    def cleanup(self):
        self.stop_motors()