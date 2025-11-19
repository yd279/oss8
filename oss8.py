#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
# ev3.speaker.beep()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
right_sensor = ColorSensor(Port.S4)
left_sensor = ColorSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, 55.5, 104)

# BLACK = 9
# WHITE = 85
# threshold = (BLACK + WHITE)/2

# DRIVE_SPEED = 100
# PROPORTIONAL_GAIN = 1.2

# while True:
#     deviation = right_sensor.reflection() - threshold
#     turn_rate = PROPORTIONAL_GAIN*deviation
#     robot.drive(DRIVE_SPEED, turn_rate)
#     wait(10)

threshold = 50
kp = 1
#1
ev3.speaker.beep()
for i in range(1):
    while True:
        left_reflection = left_sensor.reflection()
        right_reflection = right_sensor.reflection()
        if right_reflection < 30:
            robot.stop()
            break
        else:
            error=left_reflection - threshold
            turn_rate = kp*error
            robot.drive(120, turn_rate)
            wait(10)

    while True:
        left_reflection = left_sensor.reflection()
        right_reflection = right_sensor.reflection()
        if right_reflection > 30:
            robot.stop()
            break
        else:
            error=left_reflection - threshold
            turn_rate = kp*error
            robot.drive(120, turn_rate)
        wait(10)
robot.straight(50)

now_dir = 1
target_dir = 4

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
#2
ev3.speaker.beep()
for i in range(2):
    while True:
        left_reflection = left_sensor.reflection()
        right_reflection = right_sensor.reflection()
        if left_reflection < 30:
            robot.stop()
            break
        else:
            error=right_reflection - threshold
            turn_rate = kp*error
            robot.drive(120, turn_rate)
            wait(10)

    while True:
        left_reflection = left_sensor.reflection()
        right_reflection = right_sensor.reflection()
        if left_reflection > 30:
            robot.stop()
            break
        else:
            error=right_reflection - threshold
            turn_rate = kp*error
            robot.drive(120, turn_rate)
        wait(10)
# robot.straight(20)

now_dir = 4
target_dir = 1

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
#3
ev3.speaker.beep()
for i in range(3):
    while True:
        left_reflection = left_sensor.reflection()
        right_reflection = right_sensor.reflection()
        if right_reflection < 30:
            robot.stop()
            break
        else:
            error=left_reflection - threshold
            turn_rate = kp*error
            robot.drive(120, turn_rate)
            wait(10)

    while True:
        left_reflection = left_sensor.reflection()
        right_reflection = right_sensor.reflection()
        if right_reflection > 30:
            robot.stop()
            break
        else:
            error=left_reflection - threshold
            turn_rate = kp*error
            robot.drive(120, turn_rate)
        wait(10)
# robot.straight(50)

now_dir = 1
target_dir = 2

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
#4
ev3.speaker.beep()
for i in range(1):
    while True:
        left_reflection = left_sensor.reflection()
        right_reflection = right_sensor.reflection()
        if right_reflection < 30:
            robot.stop()
            break
        else:
            error=left_reflection - threshold
            turn_rate = kp*error
            robot.drive(120, turn_rate)
            wait(100)

    while True:
        left_reflection = left_sensor.reflection()
        right_reflection = right_sensor.reflection()
        if right_reflection > 30:
            robot.stop()
            break
        else:
            error=left_reflection - threshold
            turn_rate = kp*error
            robot.drive(120, turn_rate)
        wait(100)

wait(2000)

ev3.speaker.beep()
for i in range(1):
    while True:
        left_reflection = left_sensor.reflection()
        right_reflection = right_sensor.reflection()
        if right_reflection < 30:
            robot.stop()
            break
        else:
            error=left_reflection - threshold
            turn_rate = kp*error
            robot.drive(120, turn_rate)
            wait(100)

    while True:
        left_reflection = left_sensor.reflection()
        right_reflection = right_sensor.reflection()
        if right_reflection > 30:
            robot.stop()
            break
        else:
            error=left_reflection - threshold
            turn_rate = kp*error
            robot.drive(120, turn_rate)
        wait(100)

now_dir = 2
target_dir = 1

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
#5
ev3.speaker.beep()
while True:
    left_reflection = left_sensor.reflection()
    right_reflection = right_sensor.reflection()
    if left_reflection <15:
        robot.stop()
        break
    else:
        error=right_reflection - threshold
        turn_rate = kp*error
        robot.drive(120, turn_rate)
    wait(10)