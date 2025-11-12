#!/usr/bin/env pybricks-micropython
import time
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from BattleMods.main import Bagel_bot

bot = Bagel_bot(
    left_motor_port=Port.B,
    right_motor_port=Port.C,
    attachment_motor_port_1=Port.A,
    ultrasonic_sensor_port=Port.S1,
    touch_sensor_port=Port.S4,
    color_sensor_port=Port.S3
    
    )

bot.speaker.beep()
print("Bagel_bot initialized successfully.")
while True:
    if bot.touch_sensor.pressed():
        time.sleep(0.2)  # Debounce delay
        bot.speaker.beep()
        print("Color Sensor detected color:", bot.color_sensor.rgb())