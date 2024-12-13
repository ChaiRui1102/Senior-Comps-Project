#!/usr/bin/env pybricks-micropython
import sys
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait

# Initialize the EV3 Brick
ev3 = EV3Brick()

print("Initializing motors...")
ev3.speaker.beep()

# Configure motors and sensors
base_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE, [12, 36])
elbow_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE, [8, 40])
gripper_motor = Motor(Port.A)
elbow_sensor = ColorSensor(Port.S3)

# Set speed limits
print("Setting speed limits...")
base_motor.control.limits(speed=60, acceleration=120)
elbow_motor.control.limits(speed=60, acceleration=120)

def check_and_fix_arm_position():
    """Check if arm is in correct position and fix if needed"""
    reflection = elbow_sensor.reflection()
    if reflection < 32:  # If arm has fallen
        print("Arm not in correct position. Moving to safe position...")
        elbow_motor.run(15)  # Slowly raise arm
        while elbow_sensor.reflection() < 32:
            wait(10)
        elbow_motor.reset_angle(0)
        elbow_motor.hold()
        print("Arm moved to safe position")
        return True
    return False

# Get position from command line argument
try:
    target_position = int(sys.argv[1])
    current_position = int(sys.argv[2])
    print("Current position: {}, Target position: {}".format(current_position, target_position))
    
    # Check and fix arm position BEFORE any base movement
    check_and_fix_arm_position()
    
    if target_position != current_position:
        print("1. Moving base to position...")
        # Calculate relative movement needed
        movement = target_position - current_position
        
        print("Moving {} degrees".format(movement))
        base_motor.run_angle(60, movement)
        wait(1000)
    else:
        print("Same as current position, skipping base movement")
    
    print("2. Lowering arm...")
    elbow_motor.run_target(60, -40)
    wait(1000)
    
    print("3. Opening gripper...")
    gripper_motor.run_target(200, -90)
    wait(1000)
    
    print("4. Raising arm...")
    elbow_motor.run_target(60, 0)
    wait(1000)
    
    print("Release operation completed")
    ev3.speaker.beep()

except Exception as e:
    print("Error during release operation: {}".format(e))
    ev3.speaker.beep(frequency=200, duration=1000)  # Error tone