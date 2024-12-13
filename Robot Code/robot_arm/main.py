#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait

# Initialize the EV3 Brick
ev3 = EV3Brick()

# Configure the gripper motor on Port A with default settings.
gripper_motor = Motor(Port.A)

# Configure the elbow motor
elbow_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE, [8, 40])

# Configure the motor that rotates the base
base_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE, [12, 36])

# Set up the Touch Sensor and Color Sensor
base_switch = TouchSensor(Port.S1)
elbow_sensor = ColorSensor(Port.S3)

def initialize_robot():
    """Initialize all motors and sensors"""
    # Set motor limits
    elbow_motor.control.limits(speed=60, acceleration=120)
    base_motor.control.limits(speed=60, acceleration=120)

    # Initialize elbow
    elbow_motor.run_time(-30, 1000)
    elbow_motor.run(15)
    while elbow_sensor.reflection() < 32:
        wait(10)
    elbow_motor.reset_angle(0)
    elbow_motor.hold()

    # Initialize base
    base_motor.run(-60)
    while not base_switch.pressed():
        wait(10)
    base_motor.reset_angle(0)
    base_motor.hold()

    # Initialize gripper
    gripper_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)
    gripper_motor.reset_angle(0)
    gripper_motor.run_target(200, -90)

    # Indicate initialization complete
    for i in range(3):
        ev3.speaker.beep()
        wait(100)

def robot_pick(position):
    """Pick up object at given position"""
    # Rotate to the pick-up position
    base_motor.run_target(60, position)
    # Lower the arm
    elbow_motor.run_target(60, -40)
    # Close the gripper
    gripper_motor.run_until_stalled(200, then=Stop.HOLD, duty_limit=50)
    # Raise the arm
    elbow_motor.run_target(60, 0)

def robot_release(position):
    """Release object at given position"""
    # Rotate to the drop-off position
    base_motor.run_target(60, position)
    # Lower the arm
    elbow_motor.run_target(60, -40)
    # Open the gripper
    gripper_motor.run_target(200, -90)
    # Raise the arm
    elbow_motor.run_target(60, 0)

# Only run initialization if this is the main program
if __name__ == "__main__":
    initialize_robot()