#!/usr/bin/env python3

import cv2
import time
from servo_controller import ServoController
from vision_processor import VisionProcessor
from motor_controller import MotorController
from config import *

class RobotController:
    def __init__(self):
        print("Initializing robot subsystems...")
        
        try:
            self.servo = ServoController()
            print("Servo controller initialized")
            
            self.vision = VisionProcessor()
            print("Vision processor initialized")
            
            self.motors = MotorController()
            print("Motor controller initialized")
            
            print("Robot initialization complete!")
            
        except Exception as e:
            print(f"Error during initialization: {e}") #check if this variable works, fail something probably
            self.cleanup()
            raise
    
    def run(self):
        print("Starting line following, pls help")
        print("Press 'q' to exit.")
        
        try:
            while True:
                #gets current frame from camera and applies our custom preprocessing pipeline
                #includes roi cropping and clahe as well
                frame = self.vision.capture_and_preprocess()
                line_found, line_center, cross_junction, binary = self.vision.detect_line(frame)
                
                #red detection
                red_detected, red_mask = self.vision.detect_red_objects(frame)
                
                #control decisions
                status = self.make_control_decision(
                    red_detected, cross_junction, line_found, line_center
                )
                
                #adds visual overlays and displays 
                self.vision.add_visual_overlays(frame, status)
                self.display_debug_windows(frame, binary, red_mask)
                
                #exit condition (our white paper xD)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            print("\nReceived keyboard interrupt")
        except Exception as e:
            print(f"Error in main loop: {e}")
        finally:
            self.cleanup()
    
    def make_control_decision(self, red_detected, cross_junction, line_found, line_center):
        
        if red_detected:
            #stopping
            self.motors.stop_motors()
            self.servo.center_servo()
            return "RED DETECTED - STOPPED"
            
        elif cross_junction:
            #going straight
            self.motors.move_forward()
            self.servo.center_servo()
            self.servo.reset_pid()  #resetting pid for clean transition
            return "CROSS JUNCTION - GO STRAIGHT"
            
        elif line_found:
            #einfach line follow
            steering_angle = self.servo.calculate_pid_steering(line_center)
            self.servo.set_angle(steering_angle)
            self.motors.move_forward()
            return f"LINE FOLLOWING: center={line_center:.2f} angle={steering_angle:.1f}"
            
        else:
            #no line,
            self.motors.stop_motors()
            self.servo.center_servo()
            return "NO LINE"
    
    def display_debug_windows(self, frame, binary, red_mask):
        cv2.imshow("Processing View", frame)
        cv2.imshow("Binary", binary)
        cv2.imshow("Red Mask", red_mask)
    
    def cleanup(self):
        print("Cleaning up robot subsystems...")
        
        try:
            if hasattr(self, 'motors'):
                self.motors.cleanup()
                print("Motors cleaned up")
        except:
            pass
            
        try:
            if hasattr(self, 'servo'):
                self.servo.cleanup()
                print("servo cleaned up")
        except:
            pass
            
        try:
            if hasattr(self, 'vision'):
                self.vision.cleanup()
                print("vision cleaned up")
        except:
            pass
        
        print("Cleanup complete!")

def main():
    try:
        robot = RobotController()
        robot.run()
    except Exception as e:
        print(f"Failed to start robot: {e}")

if __name__ == "__main__":
    main()