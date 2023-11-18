# LEGO type:standard slot:1
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import time

hub = PrimeHub()
front_motor = Motor('D')
back_motor = Motor('C')
motor_pair = MotorPair('A', 'E')
left_motor = Motor('A')
right_motor = Motor('E')
motors = motor_pair
## ----
ONE_ROTATION_DISTANCE = 27.018

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

## ----



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

def gyro_straight(motion_sensor: MotionSensor, motor_pair: MotorPair, dist: float, single_motor:Motor, speed: int):
    motor_pair.move_tank(amount=dist, unit='cm', left_speed=speed, right_speed=speed)

def barnarnar_boat():
    #gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-27, single_motor=left_motor, speed=40)
    gyro_straight_2(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=40, single_motor=left_motor, motors_power=50, direction=BACKWARD)
    back_motor.run_for_seconds(1,-30)
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=4, single_motor=left_motor, speed=40)
    angle_turn(motor_pair=motors, steer=-100, speed=30, angle=-26, stop=True)

    back_motor.run_for_seconds(1, 50)
    angle_turn(motor_pair=motors, steer=50 , speed=40, angle=1, stop=True, reset_angle=False)
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=15, single_motor=left_motor, speed=35)

# LEGO type:standard slot:1
barnarnar_boat()

