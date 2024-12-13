#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Stop, Direction
from pybricks.tools import wait

# Initialize the EV3 Brick
ev3 = EV3Brick()

print("Starting initialization...")
ev3.speaker.beep()

# Configure motors
gripper_motor = Motor(Port.A)
elbow_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE, [8, 40])
base_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE, [12, 36])

# Set up sensors
base_switch = TouchSensor(Port.S1)
elbow_sensor = ColorSensor(Port.S3)

# Set speed limits
elbow_motor.control.limits(speed=60, acceleration=120)
base_motor.control.limits(speed=60, acceleration=120)

try:
    # Initialize elbow
    print("Initializing elbow...")
    elbow_motor.run_time(-30, 1000)
    elbow_motor.run(15)
    while elbow_sensor.reflection() < 32:
        wait(10)
    elbow_motor.reset_angle(0)
    elbow_motor.hold()

    # Initialize base to touch sensor
    print("Initializing base to touch sensor...")
    base_motor.run(-60)
    while not base_switch.pressed():
        wait(10)
    base_motor.hold()
    
    # Move 20 degrees forward from touch sensor position
    print("Moving to zero position (20 degrees from touch sensor)...")
    base_motor.reset_angle(0)  # Reset angle at touch sensor position
    base_motor.run_target(60, 20)  # Move 20 degrees forward
    base_motor.reset_angle(0)  # Reset angle again to make this the new zero
    base_motor.hold()

    # Initialize gripper
    print("Initializing gripper...")
    gripper_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)
    gripper_motor.reset_angle(0)
    gripper_motor.run_target(200, -90)

    print("Initialization complete")
    # Play three beeps to indicate completion
    for i in range(3):
        ev3.speaker.beep()
        wait(100)

except Exception as e:
    print("Error during initialization: {}".format(e))
    ev3.speaker.beep(frequency=200, duration=1000)  # Error tone