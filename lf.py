# LEGO type:standard slot:15 autostart
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()
hub.light_matrix.show_image('HAPPY')

left_color_sensor = ColorSensor('B')
right_color_sensor = ColorSensor('F')
motors = MotorPair('A', 'E')
left_motor = Motor('A')

# calculates turn by taking error and multiplying it by + or - and KColor 
# to find the direction and magnitude of correction
#. 
def calculate_turn_for_color(error: int, black_on_left: bool):
    KColor = 1.0
    magnitude = int(error * KColor)
    if black_on_left is True:
        magnitude = magnitude * -1
    
    print("calculate_turn err=", error, ", mag=", magnitude)
    return magnitude

def is_right_black(color_sensor: ColorSensor):
    right_color = color_sensor.get_reflected_light()
    if right_color <= 60:
        return True
    else:
        return False

def calculate_steering_for_gyro(error: int, total_error: int)-> int:
    KAngle = 1.10
    KTotalError = 0.15
    # steering can be at most 100
    magnitude = int((KAngle * error)  + (total_error * KTotalError))
    mag = min(100, abs(magnitude))
    if magnitude < 0:
        mag = mag * -1
    return mag * -1

def gyro_turn(motion_sensor: MotionSensor, motor_pair: MotorPair, degrees: int, turn_speed=25, left_turn=False):
    if turn_speed < 20:
        raise ValueError("turn speed < 20")
    
    if degrees < 0:
        raise ValueError("degrees < 0. Use left_turn=True")

    half_power = int(turn_speed/2)
    if left_turn:  # turning left
        half_power = -1 * half_power

    motor_pair.start_tank(half_power, -1*half_power)
    motion_sensor.reset_yaw_angle()

    def turn_complete():
        return abs(motion_sensor.get_yaw_angle()) >= degrees

    wait_until(turn_complete)
    motor_pair.stop()

def gyro_straight(motion_sensor: MotionSensor, motor_pair: MotorPair, dist_degrees: int, single_motor:Motor, motors_power=25):
    
    backwards = False
    if motors_power < 0:
        backwards = True

    motion_sensor.reset_yaw_angle()
    single_motor.set_degrees_counted(0)

    start_degrees = single_motor.get_degrees_counted()
    print('Starting Gyro Straight deg', start_degrees)
    total_error = 0

    degrees_traveled = 0
    while  abs(degrees_traveled) < dist_degrees:
        gyro_error = motion_sensor.get_yaw_angle()
        total_error = total_error + gyro_error

        if gyro_error == 0:
            total_error = 0

        steering =  calculate_steering_for_gyro(error=gyro_error, total_error=total_error)
        if backwards:
            steering = -1 * steering
        current_degrees = single_motor.get_degrees_counted()
        print('error', gyro_error, 'steering', steering, 'deg', current_degrees)
        motors.start_at_power(motors_power, steering)
        
        degrees_traveled = current_degrees - start_degrees

    motor_pair.stop()


# black_on_left: True if black line is on the left of white.
def line_follow(color_sensor: ColorSensor, motors: MotorPair, black_on_left: bool, 
                dist_degrees: int, single_motor: Motor, stop_color_sensor:ColorSensor):
    print ("            ")
    print (" *** Staring line_follow")

    target_reflected = 80
    motors_power = 20
    start_degrees = single_motor.get_degrees_counted()
    degrees_traveled = 0

    while  degrees_traveled < dist_degrees:
        color_error = color_sensor.get_reflected_light() - target_reflected
        steering = calculate_turn_for_color(color_error, black_on_left=True)
        motors.start_at_power(motors_power, steering)

        current_degrees = single_motor.get_degrees_counted()
        degrees_traveled = current_degrees - start_degrees
        if is_right_black(stop_color_sensor) is True:
            break

    motors.stop()

#line_follow(left_color_sensor, motors, black_on_left=True,
#    dist_degrees=360, single_motor=left_motor,
#    stop_color_sensor=right_color_sensor)

gyro_turn(motion_sensor=hub.motion_sensor, motor_pair=motors, degrees=90, turn_speed=25, left_turn=False)
gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist_degrees=800, single_motor=left_motor, motors_power=-25)
