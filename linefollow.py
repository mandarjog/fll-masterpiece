from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()
hub.light_matrix.show_image('HAPPY')


left_color_sensor = ColorSensor('B')
right_color_sensor = ColorSensor('F')
motors = MotorPair('A', 'E')
left_motor = Motor('A')
ONE_ROTATION_DISTANCE = 27.018
# calculates turn by taking error and multiplying it by + or - and KColor
# to find the direction and magnitude of correction


#DRIVE FUNCTIONS
def wait_for_yaw(angle=90):
    yaw = 0
    if angle > 0:
        while yaw <= angle: yaw = hub.motion_sensor.get_yaw_angle()
    elif angle < 0:
        while yaw >= angle: yaw = hub.motion_sensor.get_yaw_angle()


def angle_turn(motor_pair=motors, steer=100, speed=50, angle=90, stop=False):
    hub.motion_sensor.reset_yaw_angle()
    motor_pair.start(steering=steer, speed=speed)
    wait_for_yaw(angle=angle)
    if stop:
        motor_pair.stop()


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
    KAngle = 2
    KTotalError = 0.5
    magnitude = int((KAngle * error)+ (total_error * KTotalError)) * -1
    return magnitude


def calculate_distance(degress_traveled: float): 
      return (degress_traveled * ONE_ROTATION_DISTANCE) / 360

def gyro_straight(motion_sensor: MotionSensor, motor_pair: MotorPair, dist: float, single_motor:Motor):
    print('Starting Gyro Straight')
    motion_sensor.reset_yaw_angle()
    start_degrees = single_motor.get_degrees_counted()
    motors_power = 20
    
    total_error = 0
    dist_degrees = dist * 360 / ONE_ROTATION_DISTANCE
    print("dist_degrees:", dist_degrees)

    degrees_traveled = 0
    if dist > 0:
        while degrees_traveled < dist_degrees:
            gyro_error = motion_sensor.get_yaw_angle()
            total_error = total_error + gyro_error

            if gyro_error == 0:
                total_error = 0

            steering =calculate_steering_for_gyro(error=gyro_error, total_error=total_error)
            print('error', gyro_error, 'steering', steering)
            motors.start_at_power(motors_power, steering)
            current_degrees = single_motor.get_degrees_counted()
            degrees_traveled = start_degrees - current_degrees
            print("degrees_traveled:",degrees_traveled, "distance traveled", calculate_distance(degrees_traveled))
    else: 
        motors_power = -20

        while degrees_traveled > dist_degrees:
            gyro_error = motion_sensor.get_yaw_angle()
            total_error = total_error + gyro_error

            if gyro_error == 0:
                total_error = 0

            steering =calculate_steering_for_gyro(error=gyro_error, total_error=total_error)
            print('reverse error', gyro_error, 'steering', steering*-1)
            motors.start_at_power(motors_power, steering*-1)
            current_degrees = single_motor.get_degrees_counted()
            degrees_traveled =  start_degrees - current_degrees
            print("reverse degrees_traveled:",degrees_traveled, "distance traveled", calculate_distance(degrees_traveled))


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

    while degrees_traveled < dist_degrees:
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
gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=59, single_motor=left_motor)
angle_turn(motor_pair=motors, steer=-100, speed=50, angle= -20, stop=True)
gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=1.8, single_motor=left_motor)
gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-3, single_motor=left_motor)
gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=3, single_motor=left_motor)
gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-3, single_motor=left_motor)
gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=3, single_motor=left_motor)
gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-2.5, single_motor=left_motor)
# gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=2, single_motor=left_motor)
# gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-2, single_motor=left_motor)
# gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=2, single_motor=left_motor)
# gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-2, single_motor=left_motor)
#angle_turn(motor_pair=motors, steer=100, speed=50, angle=90, stop=True)
