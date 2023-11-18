
# LEGO type:standard slot:7

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

def mission_10_rolling_camera():
    """
    connor
    rolling_camera
    """
    hub.light_matrix.show_image('PACMAN')

    left_motor.run_to_position(0)
    right_motor.run_to_position(0)
    wait_for_seconds(5)

    motor_pair.move_tank(-30, 'cm', left_speed=-50, right_speed=-50)
    motor_pair.move_tank(-25, 'cm', left_speed=50, right_speed=50)

mission_10_rolling_camera()