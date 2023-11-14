# LEGO type:standard slot:2 autostart

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

##

hub = PrimeHub()
front_motor = Motor('D')
back_motor = Motor('C')
motors = MotorPair('A', 'E')
left_motor = Motor('A')
right_motor = Motor('E')
front_motor = Motor('D')


##

def mission_1_cinema_dragon(motor_pair=motors):
    motor_pair = MotorPair('A', 'E')
    front_motor = Motor('C')
    motor_pair.move_tank(16, 'cm', left_speed=-20, right_speed=-20)
    front_motor.run_for_seconds(1, speed=-50)
    motor_pair.move_tank(16, 'cm', left_speed =20, right_speed=20)
    front_motor.run_to_position(0)

mission_1_cinema_dragon(motors)

