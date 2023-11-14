
# LEGO type:standard slot:6 autostart

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

##

hub = PrimeHub()
front_motor = Motor('D')
back_motor = Motor('C')
left_motor = Motor('A')
right_motor = Motor('E')
motor_pair = MotorPair('A', 'E')

##

def mission_10(motor_pair, front_motor):
    """
    connor
    sounds mixer
    """
    motor_pair.start_tank(-10, -10)
    motor_pair.move_tank(13, 'cm', left_speed=-20, right_speed=-20)
    front_motor.run_to_position(208)
    motor_pair.move_tank(13, 'cm', left_speed =20, right_speed=20)
    front_motor.run_to_position(0)

mission_10(motors, front_motor)
