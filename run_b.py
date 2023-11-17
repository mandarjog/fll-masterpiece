# LEGO type:standard slot:2 autostart

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import time
motors = MotorPair('A', 'E')
left_motor = Motor('A')
hub = PrimeHub()
ONE_ROTATION_DISTANCE = 27.018
## ----

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

## ---


def mission_1_cinema_dragon():
    motor_pair = MotorPair('A', 'E')
    front_motor = Motor('C')

    #motor_pair.move_tank(16, 'cm', left_speed=-20, right_speed=-20)
    gyro_straight_2(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=23, single_motor=left_motor, motors_power=50, direction=BACKWARD)
    front_motor.run_for_seconds(1, speed=-50)
    gyro_straight_2(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=22, single_motor=left_motor, motors_power=50, direction=FORWARD)
    #motor_pair.move_tank(16, 'cm', left_speed =20, right_speed=20)
    front_motor.run_to_position(0)

# LEGO type:standard slot:2
mission_1_cinema_dragon()

