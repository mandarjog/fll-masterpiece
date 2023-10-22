from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()

hub.light_matrix.show_image('HAPPY')

hub = PrimeHub()
color_sensor = ColorSensor('B')
motors = MotorPair('A', 'E')
left_motor = Motor('A')

# calculates turn by taking error and multiplying it by + or - and KColor 
# to find the direction and magnitude of correction
#. 
def calculate_turn_for_color(error, black_on_left):
    KColor = 1.0
    magnitude = int(error * KColor)
    if black_on_left is True:
        magnitude = magnitude * -1
    
    print("calculate_turn err=", error, ", mag=", magnitude)
    return magnitude


# black_on_left: True if black line is on the left of white.
def line_follow(color_sensor, motors, black_on_left, dist_degrees, single_motor):
    print ("            ")
    print (" *** Staring line_follow")

    target_reflected = 80
    motors_power = 20
    start_degrees = single_motor.get_degrees_counted()
    degrees_traveled = 0

    while  degrees_traveled < dist_degrees:
        error = color_sensor.get_reflected_light() - target_reflected
        steering = calculate_turn_for_color(error, True)
        motors.start_at_power(motors_power, steering)

        current_degrees = single_motor.get_degrees_counted()
        degrees_traveled = current_degrees - start_degrees

line_follow(color_sensor, motors, black_on_left=True, dist_degrees=360, single_motor=left_motor)
