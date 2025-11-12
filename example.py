#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math
import time


class Bagel_bot(EV3Brick):
    def __init__(self, 
    left_motor_port, 
    right_motor_port,
    attachment_motor_port_1=None,
    attachment_motor_port_2=None,
    color_sensor_port=None,
    touch_sensor_port=None,
    ultrasonic_sensor_port=None,
    gyro_sensor_port=None,
    ):
        super().__init__()
        self.left_motor = Motor(left_motor_port)
        
        self.right_motor = Motor(right_motor_port)
        
        # Create DriveBase - wheel_diameter and axle_track are in millimeters
        # Try positional args in order: (left_motor, right_motor, wheel_diameter, axle_track)
        try:
            print("Attempting to create DriveBase with positional args...")
            self.drive_base = DriveBase(self.left_motor, self.right_motor, 56, 114)
        except Exception as e:
            # Re-raise with more context so the EV3 log shows helpful details
            print("Failed to create DriveBase:", e)
            print("Left motor details:", self.left_motor)
            print("Right motor details:", self.right_motor)
            raise
        # Some versions of DriveBase may not implement reset(); avoid calling it here
        print("Drive base initialized.")

        self.attachment_motor_port_1 = attachment_motor_port_1
        self.attachment_motor_port_2 = attachment_motor_port_2

        if attachment_motor_port_1:
            print("Attachment motor 1 initialized.")
            self.attachment_motor_1 = Motor(attachment_motor_port_1)
        if attachment_motor_port_2:
            print("Attachment motor 2 initialized.")
            self.attachment_motor_2 = Motor(attachment_motor_port_2)
        if color_sensor_port:
            print("Color sensor initialized.")
            self.color_sensor = ColorSensor(color_sensor_port)
        if touch_sensor_port:
            print("Touch sensor initialized.")
            self.touch_sensor = TouchSensor(touch_sensor_port)
        if ultrasonic_sensor_port:
            print("Ultrasonic sensor initialized.")
            self.ultrasonic_sensor = UltrasonicSensor(ultrasonic_sensor_port)
        if gyro_sensor_port:
            print("Gyro sensor initialized.")
            self.gyro_sensor = GyroSensor(gyro_sensor_port)
            self.gyro_sensor.reset_angle()

    def configure_settings(self, speed, acceleration, turnRate, turnAcceleration):
        self.drive_base.settings(speed, acceleration, turnRate, turnAcceleration)

    def move(self, distance):
        self.drive_base.straight(distance)

    def turn(self, angle):
        self.drive_base.turn(angle)

    def move_attachment(self, motor_port, speed, angle, stopType=Stop.COAST, wait=False):
        if motor_port == self.attachment_motor_port_1:
            self.attachment_motor_1.run_angle(speed, angle, then=stopType, wait=wait)
        elif motor_port == self.attachment_motor_port_2:
            self.attachment_motor_2.run_angle(speed, angle, then=stopType, wait=wait)
        else:
            raise ValueError("Invalid attachment motor port")

    def pressed(self):
        if hasattr(self, 'touch_sensor'):
            return self.touch_sensor.pressed()
        return None

    def distance(self):
        if hasattr(self, 'ultrasonic_sensor'):
            return self.ultrasonic_sensor.distance()
        return None

    def color(self):
        # Return one of the Color constants (Color.RED, Color.BLUE, Color.GREEN, Color.BROWN)
        if not hasattr(self, 'color_sensor'):
            return None

        current = self.color_sensor.rgb()
        try:
            r, g, b = current
        except Exception:
            return None

        # Prototype RGB values for target colors (tuned from observations)
        prototypes = {
            Color.RED:   (44, 12, 18),
            Color.BLUE:  (5, 8, 23),
            Color.GREEN: (10, 19, 8),
            Color.BROWN: (11, 9,  10),
        }

        def dist2(a, b):
            return (a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2

        best = None
        best_d = None
        for col, proto in prototypes.items():
            d = dist2((r, g, b), proto)
            if best is None or d < best_d:
                best = col
                best_d = d

        # Accept only if within a reasonable squared-distance threshold.
        # Threshold 600 corresponds to RMS error ~24 per channel (adjust if needed).
        if best_d is not None and best_d <= 600:
            return best
        return None


    def angle(self):
        if hasattr(self, 'gyro_sensor'):
            return self.gyro_sensor.angle()
        return None

    def beep(self):
        self.speaker.beep()


### Your code goes here ###

# Initialise the robot with motors and sensors
robot = Bagel_bot(
    left_motor_port=Port.B,
    right_motor_port=Port.C,
    attachment_motor_port_1=Port.A,
    ultrasonic_sensor_port=Port.S1,
    touch_sensor_port=Port.S4,
    color_sensor_port=Port.S3
    
    )

# while True: # Loop forever
#     if robot.distance()<100: # If the ultrasonic sensor detects an object within 100 mm
#         robot.move(-200) # Move backward 200 mm
#         wait(1000) # Wait for 1 second
#         robot.turn(180) # Turn around
#     robot.move(500) # Move forward 500 mm
#     robot.move_attachment(motor_port=Port.A, speed=500, angle=90) # Move attachment motor

robot.beep()
while True:
    if robot.color() == Color.RED:
        robot.speaker.say("Red color detected")
    elif robot.color() == Color.BLUE:
        robot.speaker.say("Blue color detected")
    elif robot.color() == Color.GREEN:
        robot.speaker.say("Green color detected")
    elif robot.color() == Color.BROWN:
        robot.speaker.say("Brown color detected")
    else:
        robot.beep()