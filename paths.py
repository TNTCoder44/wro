from robot import Robot
from pybricks.tools import wait

from scanner import Colors

import constants as constants

robot = Robot()

samples = []

def start_routine(): 
    print(robot.hub.battery.voltage()/1000.0, "V")
    robot.hub.imu.reset_heading(0)

    start_rover()
    remove_ball()
    #samples_routine()
    #score_drone()
    #deliver_samples()

def start_rover():
    dist = 170 # test on field; in mm

    robot.drive.straight_line_distance(180, 50)
    robot.drive.turn_angle(-90)
    robot.drive.straight_reflection_end(50)
    robot.drive.turn_angle(0)
    robot.drive.straight_line_distance(dist, 50)

    robot.arm.move_front_arm(120, wait=True)
    robot.drive.straight_distance(-dist + 20, 50)
    robot.arm.move_front_arm(80, wait=True)
    
    robot.drive.turn_angle(180)
    
def remove_ball():
    robot.arm.move_front_arm(129, wait=True)
    robot.drive.straight_distance(150, 45)
    robot.drive.straight_distance(-75, 30)
    robot.drive.straight_distance(100, 20)
    robot.drive.straight_distance(-100, 5)

def samples_routine(): 
    global samples 

    robot.arm.move_front_arm(50, wait=True)
    robot.drive.straight_reflection_start(50)
    robot.drive.straight_distance(160, 50)
    robot.drive.turn_angle(90)
    robot.drive.straight_time(700, -50)

    robot.hub.imu.reset_heading(0)
    samples = robot.drive.straight_scanner(1000, 50)
    print(samples)

def score_drone():
    robot.drive.straight_distance(-50, 50)
    robot.drive.turn_angle(90, wheel="left")
    robot.drive.straight_distance(800, 80)
    robot.drive.straight_distance(-1000, 80)
    robot.drive.turn_angle(180)
    robot.drive.straight_time(500, -50)

def deliver_samples_top():
    robot.hub.imu.reset_heading(0)
    robot.drive.straight_distance(-150, 50)

    robot.drive.turn_angle(90)
    robot.drive.straight_reflection_start(50)
    robot.drive.straight_line_distance(20, 50, "right")

    delivery_dist = 140

    var = 22

    robot.drive.turn_angle(-105)

    robot.drive.straight_distance(-delivery_dist, 50)
    robot.arm.move_back_arm(constants.kBackDeliver, wait=True)
    robot.drive.straight_distance(var, 50)
    robot.arm.move_back_arm(constants.kBackUp, wait=True)
    robot.drive.straight_distance(delivery_dist - var, 50)
    
    
    robot.drive.turn_angle(-80)

    robot.drive.straight_distance(-delivery_dist, 50)
    robot.arm.move_back_arm(constants.kBackDown, wait=True) ### on purpose backdown, because green is very low and likes to be stuck
    robot.drive.straight_distance(var, 50)
    robot.drive.straight_distance(delivery_dist - var, 50)
    robot.arm.move_back_arm(constants.kBackUp, wait=True)

    robot.drive.turn_angle(-90)

def deliver_samples_bottom():
    robot.hub.imu.reset_heading(0)
    robot.drive.straight_distance(50, 50)

    robot.drive.turn_angle(90)
    robot.drive.straight_reflection_start(50)
    robot.drive.straight_line_distance(20, 50, "right")

    delivery_dist = 140

    robot.drive.turn_angle(-100)

    var = 22

    robot.drive.straight_distance(-delivery_dist, 50)
    robot.arm.move_back_arm(constants.kBackDeliver, wait=True)
    robot.drive.straight_distance(var, 50)
    robot.arm.move_back_arm(constants.kBackUp, wait=True)
    robot.drive.straight_distance(delivery_dist - var, 50)
    
    
    robot.drive.turn_angle(-75)

    delivery_dist += 25

    robot.drive.straight_distance(-delivery_dist, 50)
    robot.arm.move_back_arm(constants.kBackDeliver, wait=True)
    robot.drive.straight_distance(var, 50)
    robot.drive.straight_distance(delivery_dist - var, 50)
    robot.arm.move_back_arm(constants.kBackUp, wait=True)

    robot.drive.turn_angle(-90)


def deliver_samples():
    global samples
    
    robot.hub.imu.reset_heading(0)
    # reverse samples, because we start from the other side of the field
    
    # go until green sample position, then back/forward until white sample
    # finally go against wall at the top to reset imu

    between_mm = 95

    #.index sometimes throws an error; this cannot be used
    try:
        green_pos = (5 - samples.index(Colors.GREEN)) * between_mm + constants.kStartSamplesDistance
        white_pos = (5 - samples.index(Colors.WHITE)) * between_mm + constants.kStartSamplesDistance
        red_pos = (5 - samples.index(Colors.RED)) * between_mm + constants.kStartSamplesDistance
        yellow_pos = (5 - samples.index(Colors.YELLOW)) * between_mm + constants.kStartSamplesDistance

        wait_top = samples.index(Colors.WHITE) * 400 + 2000
        wait_bottom = samples.index(Colors.YELLOW) * 750 + 500
    except ValueError:
        samples = [Colors.NONE] * 6
        green_pos = (5 - samples.index(Colors.NONE)) * between_mm + constants.kStartSamplesDistance
        white_pos = (5 - samples.index(Colors.NONE)) * between_mm + constants.kStartSamplesDistance
        red_pos = (5 - samples.index(Colors.NONE)) * between_mm + constants.kStartSamplesDistance
        yellow_pos = (5 - samples.index(Colors.NONE)) * between_mm + constants.kStartSamplesDistance

        wait_top = 2000
        wait_bottom = 2000

    diff = white_pos - green_pos
    if (diff < 0):
        diff += 30
    else:
        diff -= 5

    samp_dist = 90 # test on field; in mm

    robot.drive.straight_distance(green_pos, 50)
    robot.drive.turn_angle(90)

    robot.arm.move_back_arm(constants.kBackDown, wait=True)
    robot.drive.straight_distance(-samp_dist, 70)
    robot.arm.move_back_arm(constants.kBackUp, wait=True)
    robot.drive.straight_distance(samp_dist, 70)

    robot.drive.turn_angle(0)
    robot.drive.straight_distance(diff, 50)

    robot.drive.turn_angle(90)

    #
    robot.arm.move_back_arm(constants.kBackDown, wait=True)
    robot.drive.straight_distance(-samp_dist, 70)
    robot.arm.move_back_arm(constants.kBackUp, wait=True)
    robot.drive.straight_distance(samp_dist, 70)
    #

    robot.drive.turn_angle(0)
    
    robot.drive.straight_time(wait_top, 80)

    deliver_samples_top()

    # field width
    field_width = 1150.5  # in mm

    robot.drive.straight_distance(375, 70)
    robot.drive.turn_angle(0)
    robot.drive.straight_time(1000, 70)
    robot.hub.imu.reset_heading(0)


    robot.drive.straight_distance(-(field_width-red_pos) + 230, 50)
    robot.drive.turn_angle(90)

    #
    robot.arm.move_back_arm(constants.kBackDown, wait=True)
    robot.drive.straight_distance(-samp_dist, 70)
    robot.arm.move_back_arm(constants.kBackUp, wait=True)
    robot.drive.straight_distance(samp_dist, 70)
    #

    robot.drive.turn_angle(0)

    diff = yellow_pos - red_pos
    if (diff < 0):
        diff += 25
    else:
        diff -= 5

    robot.drive.straight_distance(diff, 50)
    robot.drive.turn_angle(90)

    #
    robot.arm.move_back_arm(constants.kBackDown, wait=True)
    robot.drive.straight_distance(-samp_dist, 70)
    robot.arm.move_back_arm(constants.kBackUp, wait=True)
    robot.drive.straight_distance(samp_dist, 70)
    #

    robot.drive.turn_angle(0)

    robot.drive.straight_time(wait_bottom, -80)

    robot.hub.imu.reset_heading(0)
    deliver_samples_bottom()


    








