from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()
#hub.light_matrix.show_image('HAPPY')

front_motor = Motor('C')
left_color_sensor = ColorSensor('B')
right_color_sensor = ColorSensor('F')

motors = MotorPair('A', 'E')
left_motor = Motor('A')
right_motor = Motor('E')

ONE_ROTATION_DISTANCE = 27.018
# calculates turn by taking error and multiplying it by + or - and KColor
# to find the direction and magnitude of correction
#.

#DRIVE FUNCTIONS
def wait_for_yaw(angle=90):
    yaw = 0
    if angle > 0:
        while yaw <= angle: yaw = hub.motion_sensor.get_yaw_angle()
    elif angle < 0:
        while yaw >= angle: yaw = hub.motion_sensor.get_yaw_angle()


def angle_turn(motor_pair=motors, steer=100, speed=50, angle=90, stop=False, reset_angle=True):
    if reset_angle:
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

def gyro_straight(motion_sensor: MotionSensor, motor_pair: MotorPair, dist: float, single_motor:Motor, speed: int):
    motor_pair.move_tank(amount=dist, unit='cm', left_speed=speed, right_speed=speed)

def gyro_straight_A(motion_sensor: MotionSensor, motor_pair: MotorPair, dist: float, single_motor:Motor, speed: int):
    print('Starting Gyro Straight')
    start_yaw_angle = motion_sensor.get_yaw_angle()
    start_degrees = single_motor.get_degrees_counted()
    motors_power = speed

    total_error = 0
    dist_degrees = dist * 360 / ONE_ROTATION_DISTANCE
    print("dist_degrees:", dist_degrees)

    if abs(speed) < 20:
        raise ValueError()

    degrees_traveled = 0
    if dist > 0:
        while degrees_traveled < dist_degrees or single_motor.was_interrupted():
            gyro_error = motion_sensor.get_yaw_angle() - start_yaw_angle
            total_error = total_error + gyro_error

            if gyro_error == 0:
                total_error = 0

            steering =calculate_steering_for_gyro(error=gyro_error, total_error=total_error)
            # print('error', gyro_error, 'steering', steering)
            motors.start_at_power(motors_power, steering)
            current_degrees = single_motor.get_degrees_counted()
            degrees_traveled = start_degrees - current_degrees
            print("degrees_traveled:",degrees_traveled, "distance traveled", calculate_distance(degrees_traveled))
    else:
        motors_power = speed * -1

        while degrees_traveled > dist_degrees: # or not single_motor.was_interrupted():
            gyro_error = motion_sensor.get_yaw_angle()
            total_error = total_error + gyro_error

            if gyro_error == 0:
                total_error = 0

            steering =calculate_steering_for_gyro(error=gyro_error, total_error=total_error)
            print('reverse error', motors_power, gyro_error, 'steering', steering*-1)
            motors.start_at_power(motors_power, steering*-1)
            current_degrees = single_motor.get_degrees_counted()
            degrees_traveled =start_degrees - current_degrees
            print("dist", dist_degrees, "<? reverse degrees_traveled:, ",degrees_traveled, ", distance traveled", calculate_distance(degrees_traveled))

    motor_pair.stop()
    return calculate_distance(degrees_traveled)


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


def mission_2_3():
    gyro_error = hub.motion_sensor.get_yaw_angle()

    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-43, single_motor=left_motor, speed=50)

    angle_turn(motor_pair=motors, steer=-100, speed=30, angle= -35, stop=True)

    motors.set_stop_action('coast')
    distance = gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-8, single_motor=left_motor, speed=30)
    print ("Total distance travelled", distance)
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=4, single_motor=left_motor, speed=30)
    angle_turn(motor_pair=motors, steer=-100, speed=30, angle= -35, stop=True, reset_angle=False)


    distance = gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-9, single_motor=left_motor, speed=30)
    print ("Total distance travelled", distance)
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=2, single_motor=left_motor, speed=30)
    angle_turn(motor_pair=motors, steer=100, speed=30, angle=90, stop=True, reset_angle=False)
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-28, single_motor=left_motor, speed=40)
    angle_turn(motor_pair=motors, steer=-100, speed=20, angle=-90, stop=True)
    #gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=2, single_motor=left_motor, speed=20)
    #motors.set_stop_action('coast')
    #motors.move(1, 'seconds', steering=0, speed=-15)
    motors.move(1, 'seconds', 0, -50)
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=4, single_motor=left_motor, speed=40)
    front_motor.run_for_degrees(-720, 100)
    wait_for_seconds(1)
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=2, single_motor=left_motor, speed=40)
    front_motor.run_for_degrees(400, 70)
    angle_turn(motor_pair=motors, steer=100, speed=30, angle=90, stop=True, reset_angle=False)


def mission_5(motor_pair, left_motor, right_motor, front_motor):
    """
    cora
    """
    # turn the robot right
    left_motor.run_for_degrees(90)
    # move robot 15 degrees foward at the speed of 50
    motor_pair.move_tank(-15, 'cm', left_speed=-50, right_speed=-50)
    # move the front attachement down
    front_motor.run_for_degrees(-180)
    # code for preparing the bot to do some real work
    # swing robot right to flick lever
    right_motor.run_for_degrees(-180)

def mission_10(motor_pair, front_motor):
    """
    connor
    """
    motor_pair.move_tank(-20, 'cm', left_speed=-50, right_speed=-50)
    front_motor.run_to_position(0)
    front_motor.run_for_degrees(300)
    motor_pair.move_tank(-2, 'cm', left_speed=-1, right_speed=-1)
    motor_pair.move_tank(5, 'cm', left_speed=10, right_speed=-10)


def mission_8(motors, left_motor, right_motor):
    """
    Rolling Camera
    @connor
    """
    left_motor.run_to_position(0)
    right_motor.run_to_position(0)
    wait_for_seconds(5)
    motors.move_tank(-30, 'cm', left_speed=-20, right_speed=-20)
    motors.move_tank(-25, 'cm', left_speed=20, right_speed=20)

mission_2_3()
mission_5(motors, left_motor, right_motor, front_motor)
