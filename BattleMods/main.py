#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


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
        if hasattr(self, 'color_sensor'):
            return self.color_sensor.color()
        return None

    def angle(self):
        if hasattr(self, 'gyro_sensor'):
            return self.gyro_sensor.angle()
        return None

    def beep(self):
        self.speaker.beep()

### Your code goes here ###
