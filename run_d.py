# LEGO type:standard slot:4

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import time

hub = PrimeHub()
#hub.light_matrix.show_image('HAPPY')
front_motor = Motor('D')
back_motor = Motor('C')
#left_color_sensor = ColorSensor('B')
#right_color_sensor = ColorSensor('F')

motors = MotorPair('A', 'E')
left_motor = Motor('A')
right_motor = Motor('E')

ONE_ROTATION_DISTANCE = 27.018
# calculates turn by taking error and multiplying it by + or - and KColor
# to find the direction and magnitude of correction
#.


## ----

def gyro_turn2(motion_sensor: MotionSensor, motor_pair: MotorPair, degrees: int, turn_speed=25, left_turn=False):
    if turn_speed < 20:
        raise ValueError("turn speed < 20")

    if degrees < 0:
        raise ValueError("degrees < 0. Use left_turn=True")

    half_power = int(turn_speed/2)
    if left_turn:# turning left
        half_power = -1 * half_power

    motor_pair.start_tank(half_power, -1*half_power)
    motion_sensor.reset_yaw_angle()

    def turn_complete():
        return abs(motion_sensor.get_yaw_angle()) >= degrees

    wait_until(turn_complete)
    motor_pair.stop()


FORWARD = "forward"
BACKWARD = "backward"

# accelerate for this number of degrees
P1 = 360
# deccelerate for this number of degrees
P2 = 360

min_power = 20

def get_power(max_power:int, dist_max: int, dist_degrees:int)-> int:
    # 0 -- p1 -- p2 -- dist_max

    power = 0
    c = 0

    if dist_degrees <= P1:
        m = (max_power - min_power)/P1
        power = m * dist_degrees + min_power
        c = 1
    elif dist_degrees > P1 and dist_degrees < (dist_max - P2):
        power = max_power
        c = 2
    else:
        m = (max_power - min_power) / (P2)
        power = min_power + m * (dist_max - dist_degrees)
        c = 3

    print("deg", dist_degrees, "power", power, c)
    return int(power)

def gyro_straight_2(
    motion_sensor: MotionSensor,
    motor_pair: MotorPair,
    dist: int,
    single_motor: Motor,
    motors_power=25,
    fix_orientation=False,
    direction=FORWARD,
    maxtime = 0.0
):
    DEBUG = True
    start_time = time.time()
    if motors_power < 20:
        raise Exception("motors_power <20 or negative", motors_power)

    start_power = 20

    dist_degrees = int(round(dist * 360 / ONE_ROTATION_DISTANCE))

    print("gyro_straight start dist", dist, "dist_degrees", dist_degrees, direction)
    dirn = 1
    if direction == BACKWARD:
        dirn = -1

    motion_sensor.reset_yaw_angle()
    single_motor.set_degrees_counted(0)

    total_error = 0
    last_error = 0
    err_diff = 0

    gyro_error = 0
    degrees_traveled = 0

    # start moving slowly
    motors.start_at_power(dirn * start_power)

    
    mp = abs(motors_power - start_power)

    while degrees_traveled < dist_degrees:
        gyro_error = motion_sensor.get_yaw_angle()
        total_error = total_error + gyro_error

        if gyro_error == 0:
            total_error = 0

        err_diff = gyro_error - last_error
        if err_diff != 0:
            hub.light_matrix.write(str(abs(gyro_error)))

        last_error = gyro_error

        steering = dirn * calculate_steering_for_gyro(
            error=gyro_error, total_error=total_error, err_diff=err_diff
        )
        degrees_traveled = abs(single_motor.get_degrees_counted())

        power = get_power(motors_power, dist_degrees, degrees_traveled)
        # power increases from start_power to motors_power and then
        # back to start_power so that breaking is gentle
        #power = int(
        #    motors_power
        #    - mp * 2 * abs(degrees_traveled - dist_degrees / 2) / dist_degrees
        #)

        # if power is less than start_power (20), the robot does not move
        power = max(start_power, power)

        if DEBUG:
            print(
                "error",
                gyro_error,
                "steering",
                steering,
                "deg",
                degrees_traveled,
                "/",
                dist_degrees,
                (degrees_traveled - dist_degrees / 2) / dist_degrees,
                "power",
                power,
            )

        motors.start_at_power(dirn * power, steering)

        if maxtime > 0:
            if time.time() - start_time > maxtime:
                print("maxtime reached")
                break

    motor_pair.stop()

    print("gyro_straight end error", gyro_error)

    if gyro_error != 0 and fix_orientation:  # fix orientation
        left_turn = True
        if gyro_error < 0:
            left_turn = False
        gyro_turn2(
            motion_sensor, motor_pair, degrees=abs(gyro_error), left_turn=left_turn
        )

## ---


def gyro_straight2(motion_sensor: MotionSensor, motor_pair: MotorPair, dist_degrees: int, single_motor:Motor, motors_power=25):

    backwards = False
    if motors_power < 0:
        backwards = True

    motion_sensor.reset_yaw_angle()
    single_motor.set_degrees_counted(0)

    start_degrees = single_motor.get_degrees_counted()
    print('Starting Gyro Straight deg', start_degrees)
    total_error = 0

    degrees_traveled = 0
    while abs(degrees_traveled) < dist_degrees:
        gyro_error = motion_sensor.get_yaw_angle()
        total_error = total_error + gyro_error

        if gyro_error == 0:
            total_error = 0

        steering =calculate_steering_for_gyro(error=gyro_error, total_error=total_error)
        if backwards:
            steering = -1 * steering
        current_degrees = single_motor.get_degrees_counted()
        print('error', gyro_error, 'steering', steering, 'deg', current_degrees)
        motors.start_at_power(motors_power, steering)

        degrees_traveled = current_degrees - start_degrees

    motor_pair.stop()



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

def calculate_steering_for_gyro1(error: int, total_error: int)-> int:
    KAngle = 1
    KTotalError = 0.25
    magnitude = int((KAngle * error)+ (total_error * KTotalError)) * -1
    return magnitude

def calculate_steering_for_gyro(error: int, total_error: int, err_diff=0) -> int:
    KAngle = 1.15
    KTotalError = 0.2
    kdiff = 0.2
    # steering can be at most 100
    magnitude = int((KAngle * error) + (total_error * KTotalError) + err_diff * kdiff)
    mag = min(100, abs(magnitude))
    if magnitude < 0:
        mag = mag * -1
    return mag * -1


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


def mission_2_3_5():
    """
    run: B
    """
    gyro_error = hub.motion_sensor.get_yaw_angle()
    #go to straight to mision 2
    hub.light_matrix.write('B')
    gyro_straight_2(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=64, single_motor=left_motor, motors_power=50, direction=BACKWARD)
    #turn towards mission 2
    hub.light_matrix.write('A')

    angle_turn(motor_pair=motors, steer=-100, speed=30, angle= -35, stop=True)

    #pump(do mision 2)
    motors.set_stop_action('coast')
    # drive towards mission 2
    distance = gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-8, single_motor=left_motor, speed=30)
    print ("Total distance travelled", distance)
    # drive away from mission 2
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=4, single_motor=left_motor, speed=30)
    #turn away from mission 2(towards mission 3)
    angle_turn(motor_pair=motors, steer=100, speed=30, angle=90, stop=True, reset_angle=False)

    #go to mission 3
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-30, single_motor=left_motor, speed=40)
    #turn into mission 3
    angle_turn(motor_pair=motors, steer=-100, speed=20, angle=-90, stop=True)
    #ram into missioon 3 to align
    motors.move(0.5, 'seconds', 0, -50)
    #go backwards to prepare to do mission
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=4, single_motor=left_motor, speed=40)
    #do mission
    back_motor.run_for_degrees(-720, 100)
    wait_for_seconds(1)
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=1, single_motor=left_motor, speed=40)
    back_motor.run_for_degrees(400, 70)
    #turn away from mission 3
    gyro_turn2(hub.motion_sensor, motors, degrees=90, turn_speed=25)

    _mission_5()

def mission_13():
    #go staright to mission 13
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=10, single_motor=left_motor, speed=40)
    #flick up the lid
    front_motor.run_for_degrees(100, 25)
    #go forward
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=3, single_motor=left_motor, speed=40)
    #go up more
    front_motor(50, 25)


def _mission_5():
    # mission 5
    hub.light_matrix.write('A')
    #drive straight towards mission
    gyro_straight(hub.motion_sensor, motors, 10, left_motor, -50)
    #turn towards mission
    hub.light_matrix.write('B')
    gyro_turn2(hub.motion_sensor, motors, degrees=48, turn_speed=25, left_turn=True)
    #go in to mision
    gyro_straight(hub.motion_sensor, motors, 13, left_motor, -50)
    #put attachment down
    back_motor.run_for_degrees(-350, 50)
    #flick mission
    gyro_turn2(hub.motion_sensor, motors, degrees=50, turn_speed=100)
    #raise attachment back up
    back_motor.run_for_degrees(300, 50)
   
    # go back home

    # align with the table edge
    gyro_turn2(hub.motion_sensor, motors, degrees=45, turn_speed=100)
    # bump into the edge
    motors.move(1, 'seconds', 0, 50)
    # go froward again
    gyro_straight(hub.motion_sensor, motors, 15, left_motor, -50)
    gyro_turn2(hub.motion_sensor, motors, degrees=90, turn_speed=100, left_turn=True)
    gyro_straight(hub.motion_sensor, motors, 45, left_motor, -50)
    gyro_turn2(hub.motion_sensor, motors, degrees=80, turn_speed=100)
    gyro_straight(hub.motion_sensor, motors, 45, left_motor, -50)


def mission_5(motor_pair, left_motor, right_motor, front_motor):
    """
    cora
    """
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



mission_2_3_5()

