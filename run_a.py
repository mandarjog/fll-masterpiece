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
front_motor = Motor('D')


##

def mission_1_cinema_dragon(motor_pair=motors):
    motor_pair = MotorPair('E', 'A')
    front_motor = Motor('D')
    motor_pair.start_tank(-10, -10)
    motor_pair.move_tank(18, 'cm', left_speed=-20, right_speed=-20)
    front_motor.run_to_position(208)
    motor_pair.move_tank(18, 'cm', left_speed =20, right_speed=20)
    front_motor.run_to_position(0)

mission_1_cinema_dragon(motors)
