from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()
#hub.light_matrix.show_image('HAPPY')
front_motor = Motor('D')
back_motor = Motor('C')


motor_pair = MotorPair('A', 'E')
left_motor = Motor('A')
right_motor = Motor('E')

ONE_ROTATION_DISTANCE = 27.018
hub.light_matrix.show_image('HAPPY')

front_motor = Motor('D')
motor_pair.move_tank(-23, 'cm', left_speed=-50, right_speed=-50)
front_motor.run_to_position(0)
front_motor.run_for_degrees(300)
motor_pair.move_tank(-3, 'cm', left_speed=-5, right_speed=-5)
front_motor.run_for_degrees(40)
motor_pair.move_tank(5, 'cm', left_speed=10, right_speed=-10) 
motor_pair.move_tank(20, 'cm', left_speed=-20, right_speed=-20)

