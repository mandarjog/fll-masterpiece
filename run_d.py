from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()

def wait_for_yaw(angle=90):
    yaw = 0
    if angle > 0:
        while yaw <= angle: yaw = hub.motion_sensor.get_yaw_angle()
    elif angle < 0:
        while yaw >= angle: yaw = hub.motion_sensor.get_yaw_angle()
def angle_turn(steer=100, speed=50, angle=90, stop=False):
    hub.motion_sensor.reset_yaw_angle()
    drive_motors_i.start(steering=steer, speed=speed)
    wait_for_yaw(angle=angle)
    if stop:
        drive_motors_i.stop()
def gyro_turn(desired_angle,speed,direction):
    prev_speed = speed
    if direction == 'right':
        x = hub.motion_sensor.get_yaw_angle()
        print(x)
        print("I am going to ",desired_angle)
        moving_motors.start_tank(speed,speed*-1)
        while desired_angle > x:
            remaining_angle = desired_angle-x
            print("My speed is changing to ",speed)
            x = hub.motion_sensor.get_yaw_angle()
            print(x)
            if desired_angle-x <= 20: 
                speed = 4
            if prev_speed != speed:
                moving_motors.start_tank(speed, speed*-1)
            prev_speed = speed
        moving_motors.stop()
        print('The final yaw angle is: ',x)
    elif direction == 'left':
        x = hub.motion_sensor.get_yaw_angle()
        print(x)
        print("I am going to ",desired_angle)
        moving_motors.start_tank(speed*-1,speed)
        while desired_angle*-1 < x:
            remaining_angle = x-desired_angle*-1
            print("My speed is changing to ",speed)
            x = hub.motion_sensor.get_yaw_angle()
            if x-desired_angle*-1 <= 20:
                speed = 4
            if prev_speed != speed:
                moving_motors.start_tank(speed*-1,speed)
        prev_speed = speed
        moving_motors.stop()
        print('The final yaw angle is: ',x)
def gyro_straight(drive_motors, dist,spd): 
    dist = float(dist)
    DISTANCE_DIVIDER = -20.8695652174
    #we got distance divider by dividing 17.25cm (the circumference of the wheel) by -360(one wheel rotation) 
    single_motor.set_degrees_counted(0)
    drive_motors.set_stop_action('brake')
    print('distance =', dist)
    print('speed=', spd)
    org_yaw_angle = hub.motion_sensor.get_yaw_angle()
    print('original yaw =', org_yaw_angle)
    distance_traveled = 0.0
    drive_motors.set_default_speed(spd)
    drive_motors.start_at_power(spd, 0)
    drive_motors.move(dist, 'cm', 0, spd)
    print('the loop has run this distance :', distance_traveled)
    drive_motors.stop()
print('------------------------------------------------------------------------------')
print('------------------------------------------------------------------------------')


def fllStandardBot():
    drive_motors_i = MotorPair('A', 'E')
    single_motor =Motor('A')
    drive_motors = MotorPair('A', 'E')
    DISTANCE_DIVIDER = -13.04
    return drive_motors_i, single_motor, drive_motors
def fllCastBot():
    drive_motors_i = MotorPair('A', 'B')
    single_motor = Motor('A')
    drive_motors = MotorPair('A', 'B')
    DISTANCE_DIVIDER = -20.8695652174
    return drive_motors_i, single_motor, drive_motors

drive_motors_i, single_motor, drive_motors = fllStandardBot()
drive_motors.set_stop_action('brake')
moving_motors = drive_motors

def delivery_and_hologram_performer():
    gyro_straight(drive_motors, 17, 45)#drive motors, distance, speed
    angle_turn(steer=-30, speed=25, angle=-90, stop=True)#(for left, angle and steer are negative)
    gyro_straight(drive_motors, 20, 25)#drive motors, distance, speed
    angle_turn(steer=100, speed=25, angle=55, stop=True)#for right, angle and steer are positive
    gyro_straight(drive_motors, 3, 45)
    gyro_straight(drive_motors, 10, -25)
    angle_turn(steer=100, speed=25, angle=85, stop=True)
    gyro_straight(drive_motors, 35, 45)
def craft_creator():
    gyro_straight(drive_motors, -5, 45)
    angle_turn(steer=100, speed=30, angle=90, stop=True)
    gyro_straight(drive_motors, 30, 45)
    angle_turn(steer=100, speed=30, angle=136, stop=True)
    gyro_straight(drive_motors, 20, 45)
    gyro_straight(drive_motors, -30, 45)

delivery_and_hologram_performer()
craft_creator()
