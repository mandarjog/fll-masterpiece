
# LEGO type:standard slot:6 autostart

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

def mission_10(motor_pair, front_motor):
    """
    connor
    sounds mixer
    """
    motor_pair.move_tank(-20, 'cm', left_speed=-50, right_speed=-50)
    front_motor.run_to_position(0)
    front_motor.run_for_degrees(300)
    motor_pair.move_tank(-2, 'cm', left_speed=-1, right_speed=-1)
    motor_pair.move_tank(5, 'cm', left_speed=10, right_speed=-10)

mission_10(motors, front_motor)
