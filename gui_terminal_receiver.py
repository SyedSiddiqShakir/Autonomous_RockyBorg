from RockyBorg import RockyBorg
import time

# Initialize the RockyBorg interface
RB = RockyBorg()
RB.Init()
RB.SetEpoIgnore(True)  # Ignore Emergency Stop (if any)

# Loop to continuously display data
try:
    print("Reading data from RockyBorg...")
    while True:
        # Retrieve servo position
        servo_pos = RB.GetServoPosition()

        # Retrieve motor speed settings (left and right)
        left_motor_speed = RB.GetMotor1()
        right_motor_speed = RB.GetMotor2()

        # Check if manual mode is active (this would be set manually via GUI logic)
        manual_mode = True  # Simulated flag, change according to GUI state

        # Print values
        print(f"Servo Position   : {servo_pos:.2f}")
        print(f"Left Motor Speed : {left_motor_speed:.2f}")
        print(f"Right Motor Speed: {right_motor_speed:.2f}")
        print(f"Manual Mode      : {manual_mode}")
        print("-" * 40)

        time.sleep(1)  # Update every second

except KeyboardInterrupt:
    print("Stopped monitoring.")
