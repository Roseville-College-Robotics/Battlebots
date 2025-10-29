#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


class BagelBot(EV3Brick):
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
        self.left_motor = Motor(left_motor_port)
        self.right_motor = Motor(right_motor_port)
        self.drive_base = DriveBase(self.left_motor, self.right_motor, wheel_diameter=56, axle_track=114)
        self.reset()

        self.attachment_motor_port_1 = attachment_motor_port_1
        self.attachment_motor_port_2 = attachment_motor_port_2

        if attachment_motor_port_1:
            self.attachment_motor_1 = Motor(attachment_motor_port_1)
        if attachment_motor_port_2:
            self.attachment_motor_2 = Motor(attachment_motor_port_2)
        if color_sensor_port:
            self.color_sensor = ColorSensor(color_sensor_port)
        if touch_sensor_port:
            self.touch_sensor = TouchSensor(touch_sensor_port)
        if ultrasonic_sensor_port:
            self.ultrasonic_sensor = UltrasonicSensor(ultrasonic_sensor_port)
        if gyro_sensor_port:
            self.gyro_sensor = GyroSensor(gyro_sensor_port)
            self.gyro_sensor.reset_angle()

    def configureSettings(self, speed, acceleration, turnRate, turnAcceleration):
        self.settings(speed, acceleration, turnRate, turnAcceleration)

    def move(self, distance):
        self.drive_base.straight(distance)

    def turn(self, angle):
        if self.gyro_sensor:
            currentAngle = self.gyro_sensor.angle()
        else:
            currentAngle = self.angle()
        if angle > currentAngle:
            targetAngle = currentAngle + angle
        else:
            targetAngle = currentAngle - angle
        
        self.drive_base.turn(targetAngle)

    def moveAttachment(self, motor_port, speed, angle, stopType=Stop.COAST, wait=False):
        if motor_port == self.attachment_motor_port_1:
            self.attachment_motor_1.run_angle(speed, angle, stopType, wait)
        elif motor_port == self.attachment_motor_port_2:
            self.attachment_motor_2.run_angle(speed, angle, stopType, wait)
        else:
            raise ValueError("Invalid attachment motor port")

    def pressed(self):
        return self.touch_sensor.pressed()
    
    def distance(self):
        return self.ultrasonic_sensor.distance()
    
    def color(self):
        return self.color_sensor.color()
    
    def angle(self):
        return self.gyro_sensor.angle()
        

### Your code goes here ###