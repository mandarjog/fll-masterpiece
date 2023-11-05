# LEGO type:standard slot:1 autostart

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

##

hub = PrimeHub()
front_motor = Motor('D')
back_motor = Motor('C')
left_color_sensor = ColorSensor('B')
right_color_sensor = ColorSensor('F')
motors = MotorPair('A', 'E')
left_motor = Motor('A')
right_motor = Motor('E')

##

def mission_8(motors, left_motor, right_motor):
    """
    Rolling Camera
    @connor
    """
    left_motor.run_to_position(0)
    right_motor.run_to_position(0)
    wait_for_seconds(5)
    motors.move_tank(-30, 'cm', left_speed=-20, right_speed=-20)
    motors.move_tank(-25, 'cm', left_speed=20, right_speed=20)

mission_8(motors, left_motor, right_motor)

