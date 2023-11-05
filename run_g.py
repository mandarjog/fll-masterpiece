# LEGO type:standard slot:7 autostart

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

def mission_11_light_show():
    '''
    @cora fill this in.
    '''
    pass

mission_11_light_show()
