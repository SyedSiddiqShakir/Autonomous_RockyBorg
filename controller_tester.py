#!/usr/bin/env python
# coding: utf-8

# Import required libraries
import sys
import time
sys.path.insert(1, '/home/admin/project/Gamepad/')
import Gamepad
sys.path.insert(2, '/home/admin/project/rockyborg/')
import RockyBorg

#!/usr/bin/env python
import sys
import time
import RockyBorg
sys.path.insert(0, "/home/pi/Gamepad")
import Gamepad

# Xbox One Controller Configuration
gamepadType = Gamepad.XboxONE
buttonExit = 'RB'       # Changed to Xbox Home button (index 12)
buttonForward = 'Y'      # Button 4
buttonBackward = 'A'     # Button 0
buttonLeft = 'X'         # Button 3
buttonRight = 'B'        # Button 1
interval = 0.1

# Motor Configuration
voltageIn = 1.2 * 8
voltageOut = 6.0
maxPower = voltageOut / voltageIn if voltageOut <= voltageIn else 1.0

# Initialize RockyBorg
RB = RockyBorg.RockyBorg()
try:
    RB.Init()
    if not RB.foundChip:
        raise Exception("RockyBorg not found")
except Exception as e:
    print(f"Error initializing RockyBorg: {e}")
    sys.exit()

RB.SetCommsFailsafe(False)
RB.MotorsOff()
RB.SetMotorsEnabled(True)

# Gamepad setup
def exitButtonPressed():
    global running
    print("Exiting...")
    running = False

if not Gamepad.available():
    print("Please connect Xbox One controller...")
    while not Gamepad.available():
        time.sleep(0.1)

gamepad = gamepadType()
gamepad.startBackgroundUpdates()
gamepad.addButtonPressedHandler(buttonExit, exitButtonPressed)

running = True
print("RockyBorg Ready! Controls:")
print("Y = Forward, A = Backward")
print("X = Left, B = Right")
print("HOME (Xbox Button) = Exit")

try:
    while running and gamepad.isConnected():
        # Movement control
        speed = 0
        if gamepad.isPressed(buttonForward):
            speed = 1.0
        elif gamepad.isPressed(buttonBackward):
            speed = -1.0

        # Steering control
        steering = 0
        if gamepad.isPressed(buttonLeft):
            steering = -1.0
        elif gamepad.isPressed(buttonRight):
            steering = 1.0

        # Apply controls
        RB.SetMotors(speed * maxPower)
        RB.SetServoPosition(steering)
        
        time.sleep(interval)

except KeyboardInterrupt:
    running = False
finally:
    gamepad.disconnect()
    RB.MotorsOff()
    RB.SetLed(False)
    print("Controller disconnected. Motors stopped.")