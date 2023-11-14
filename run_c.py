# LEGO type:standard slot:3

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()
front_motor = Motor('D')
back_motor = Motor('C')
motor_pair = MotorPair('A', 'E')
left_motor = Motor('A')
right_motor = Motor('E')


def mission_10_sound_mixer(motor_pair, front_motor):
    """
    connor
    sounds mixer
    """
    motor_pair.move_tank(-23, 'cm', left_speed=-50, right_speed=-50)
    front_motor.run_to_position(0)
    front_motor.run_for_degrees(300)
    motor_pair.move_tank(-2, 'cm', left_speed=-5, right_speed=-5)
    front_motor.run_for_degrees(40)
    motor_pair.move_tank(5, 'cm', left_speed=10, right_speed=-10) 
    motor_pair.move_tank(10, 'cm', left_speed=-20, right_speed=-20)
    #motor_pair.move_tank(5, 'cm', left_speed=10, right_speed=0)
    #right_motor.run_to_position(90)
    motor_pair.move_tank(5, 'cm', left_speed=-100, right_speed=100)
    motor_pair.move_tank(-25, 'cm', left_speed=100, right_speed=100)

mission_10_sound_mixer(motor_pair, front_motor)