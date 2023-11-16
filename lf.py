# LEGO type:standard slot:15
from spike import (
    PrimeHub,
    MotionSensor,
    ColorSensor,
    Motor,
    MotorPair,
)
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
from hub import battery
import time

hub = PrimeHub()
hub.light_matrix.show_image("HAPPY")

front_motor = Motor("D")
back_motor = Motor("C")
motors = MotorPair("A", "E")
left_motor = Motor("A")
ONE_ROTATION_DISTANCE = 27.018


# calculates turn by taking error and multiplying it by + or - and KColor
# to find the direction and magnitude of correction
# .
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


def calculate_steering_for_gyro(error: int, total_error: int, err_diff: int) -> int:
    KAngle = 1.15
    KTotalError = 0.2
    kdiff = 0.2
    # steering can be at most 100
    magnitude = int((KAngle * error) + (total_error * KTotalError) + err_diff * kdiff)
    mag = min(100, abs(magnitude))
    if magnitude < 0:
        mag = mag * -1
    return mag * -1


def gyro_turn(
    motion_sensor: MotionSensor,
    motor_pair: MotorPair,
    degrees: int,
    turn_speed=25,
    left_turn=False,
):
    if turn_speed < 20:
        raise ValueError("turn speed < 20")

    if degrees < 0:
        raise ValueError("degrees < 0. Use left_turn=True")

    half_power = int(turn_speed / 2)
    if left_turn:  # turning left
        half_power = -1 * half_power

    motor_pair.start_tank(half_power, -1 * half_power)
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

def gyro_straight(
    motion_sensor: MotionSensor,
    motor_pair: MotorPair,
    dist: int,
    single_motor: Motor,
    motors_power=25,
    fix_orientation=False,
    direction=FORWARD,
    maxtime = 0.0
):
    DEBUG = False
    start_time = time.time()
    if motors_power < 20:
        raise ("motors_power <20 or negative", motors_power)

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
        gyro_turn(
            motion_sensor, motor_pair, degrees=abs(gyro_error), left_turn=left_turn
        )


# black_on_left: True if black line is on the left of white.
def line_follow(
    color_sensor: ColorSensor,
    motors: MotorPair,
    black_on_left: bool,
    dist_degrees: int,
    single_motor: Motor,
    stop_color_sensor: ColorSensor,
):
    print("            ")
    print(" *** Staring line_follow")

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


# line_follow(left_color_sensor, motors, black_on_left=True,
#    dist_degrees=360, single_motor=left_motor,
#    stop_color_sensor=right_color_sensor)

# gyro_turn(motion_sensor=hub.motion_sensor, motor_pair=motors, degrees=90, turn_speed=25, left_turn=False)
# gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist_degrees=1000, single_motor=left_motor, motors_power=-40)


def test_steering():
    # motors.start_at_power(power=20, steering=-50)
    motors.start_tank_at_power(left_power=20, right_power=20)


# test_steering()


def mission_2_3_5():
    """
    run: B
    """
    gyro_error = hub.motion_sensor.get_yaw_angle()
    # go to straight to mision 2
    # motion_sensor: MotionSensor, motor_pair: MotorPair,
    #              dist: int, single_motor:Motor, motors_power=25
    gyro_straight(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        dist=60,
        single_motor=left_motor,
        motors_power=60,
        fix_orientation=True,
        direction=BACKWARD,
    )
    # turn towards mission 2
    gyro_turn(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        degrees=35,
        turn_speed=25,
        left_turn=True,
    )

    # pump(do mision 2)
    motors.set_stop_action("coast")
    # drive towards mission 2
    distance = gyro_straight(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        dist=12,
        single_motor=left_motor,
        motors_power=30,
        direction=BACKWARD,
        maxtime=1
    )
    print("Total distance travelled", distance)
    # drive away from mission 2
    gyro_straight(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        dist=7,
        single_motor=left_motor,
        motors_power=30,
        fix_orientation=True,
        direction=FORWARD,
    )
    # Ensure angle is still -35, aligned towards mission 2
    # gyro_turn(motion_sensor=hub.motion_sensor, motor_pair=motors, degrees= 35, turn_speed=25, left_turn=False)
    # drive towards mission 2 - 2nd pump start
    distance = gyro_straight(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        dist=8,
        single_motor=left_motor,
        motors_power=30,
        direction=BACKWARD,
        maxtime=1
    )
    print("Total distance travelled", distance)
    # drive away from mission 2 - 2nd pump end
    gyro_straight(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        dist=3,
        single_motor=left_motor,
        motors_power=30,
        direction=FORWARD,
        fix_orientation=True,
    )
    # turn away from mission 2(towards mission 3)
    gyro_turn(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        degrees=123,
        turn_speed=30,
        left_turn=False,
    )

    # go to mission 3
    gyro_straight(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        dist=30,
        single_motor=left_motor,
        motors_power=40,
        direction=BACKWARD,
    )
    # turn into mission 3
    gyro_turn(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        degrees=90,
        turn_speed=25,
        left_turn=True,
    )

    # ram into missioon 3 to align
    motors.move(0.5, "seconds", 0, -50)
    # go backwards to prepare to do mission
    gyro_straight(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        dist=4,
        single_motor=left_motor,
        motors_power=40,
    )
    # do mission
    back_motor.run_for_degrees(-720, 100)
    wait_for_seconds(1)
    gyro_straight(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        dist=1,
        single_motor=left_motor,
        motors_power=40,
    )
    back_motor.run_for_degrees(400, 70)
    # turn away from mission 3
    gyro_turn(hub.motion_sensor, motors, degrees=90, turn_speed=25)


#mission_2_3_5()

if battery.voltage() < 8000:
    raise ("Volate less than 8000", battery.voltage())


# gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=40,
#                single_motor=left_motor, motors_power=40, fix_orientation=True, direction=BACKWARD)

motors.set_stop_action("coast")
for i in range(1):
    gyro_straight(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        dist=20,
        single_motor=left_motor,
        motors_power=25,
        fix_orientation=True,
        direction=FORWARD,
    )
    #gyro_turn(motion_sensor=hub.motion_sensor, motor_pair=motors, degrees=12)

'''
    gyro_straight(
        motion_sensor=hub.motion_sensor,
        motor_pair=motors,
        dist=20,
        single_motor=left_motor,
        motors_power=90,
        fix_orientation=True,
        direction=BACKWARD,
    )
'''
