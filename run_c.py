from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

##

hub = PrimeHub()
front_motor = Motor('D')
back_motor = Motor('C')

motors = MotorPair('A', 'E')
left_motor = Motor('A')
right_motor = Motor('E')

##

## common code
def gyro_straight(motion_sensor: MotionSensor, motor_pair: MotorPair,
                dist: float, single_motor:Motor, speed: int):
    motor_pair.move_tank(amount=dist, unit='cm', left_speed=speed, right_speed=speed)

def gyro_turn2(motion_sensor: MotionSensor, motor_pair: MotorPair,
            degrees: int, turn_speed=25, left_turn=False):
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


#DRIVE FUNCTIONS
def wait_for_yaw(angle=90):
    yaw = 0
    if angle > 0:
        while yaw <= angle: yaw = hub.motion_sensor.get_yaw_angle()
    elif angle < 0:
        while yaw >= angle: yaw = hub.motion_sensor.get_yaw_angle()


def angle_turn(motor_pair, steer=100, speed=50, angle=90, stop=False, reset_angle=True):
    if reset_angle:
        hub.motion_sensor.reset_yaw_angle()
    motor_pair.start(steering=steer, speed=speed)
    wait_for_yaw(angle=angle)
    if stop:
        motor_pair.stop()

##


def mission_2_3_5():
    """
    @sara, @cora
    """
    gyro_error = hub.motion_sensor.get_yaw_angle()
    #go to straight to mision 2
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-42, single_motor=left_motor, speed=50)
    #turn towards mission 2
    angle_turn(motor_pair=motors, steer=-100, speed=30, angle= -35, stop=True)
    #pump(do mision 2)
    motors.set_stop_action('coast')
    distance = gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-8, single_motor=left_motor, speed=30)
    print ("Total distance travelled", distance)
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=4, single_motor=left_motor, speed=30)
    angle_turn(motor_pair=motors, steer=-100, speed=30, angle= -35, stop=True, reset_angle=False)
    distance = gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-9, single_motor=left_motor, speed=30)
    print ("Total distance travelled", distance)
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=2, single_motor=left_motor, speed=30)
    #turn away from mission 2(towards mission 3)
    angle_turn(motor_pair=motors, steer=100, speed=30, angle=45, stop=True, reset_angle=False)
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-6, single_motor=left_motor, speed=40)
    angle_turn(motor_pair=motors, steer=100, speed=30, angle=90, stop=True, reset_angle=False)
    #go to mission 3
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=-26, single_motor=left_motor, speed=40)
    #turn into mission 3
    angle_turn(motor_pair=motors, steer=-100, speed=20, angle=-90, stop=True)
    #ram into missioon 3 to align
    motors.move(0.5, 'seconds', 0, -50)
    #go backwards to prepare to do mission
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=5, single_motor=left_motor, speed=40)
    #do mission
    back_motor.run_for_degrees(-720, 100)
    wait_for_seconds(1)
    gyro_straight(motion_sensor=hub.motion_sensor, motor_pair=motors, dist=1, single_motor=left_motor, speed=40)
    back_motor.run_for_degrees(400, 70)
    #turn away from mission 3
    angle_turn(motor_pair=motors, steer=100, speed=20, angle=90, stop=True)

    _mission_5()

def _mission_5():
    """
    @cora
    """
    # mission 5
    gyro_straight(hub.motion_sensor, motors, -14, left_motor, 50)
    angle_turn(motor_pair=motors, steer=-100, speed=20, angle=-50, stop=True)
    gyro_straight(hub.motion_sensor, motors, -7, left_motor, 50)
    back_motor.run_for_degrees(-300, 50)
    angle_turn(motor_pair=motors, steer=100, speed=20, angle=25, stop=True)
    # hub.light_matrix.write('A')
    # gyro_straight(hub.motion_sensor, motors, -6, left_motor, 50)
    # hub.light_matrix.write('B')
    # gyro_turn2(hub.motion_sensor, motors, degrees=50, turn_speed=25, left_turn=True)
    # gyro_straight(hub.motion_sensor, motors, 14, left_motor, 50)
    # back_motor.run_for_degrees(-300, 50)
    # gyro_turn2(hub.motion_sensor, motors, degrees=50, turn_speed=100)
#_mission_5()
# left_motor.run_to_position(0)
# right_motor.run_to_position(0)
# wait_for_seconds(2)
#_mission_5()
mission_2_3_5()
