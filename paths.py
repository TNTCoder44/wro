from robot import Robot
from pybricks.tools import wait

from scanner import Colors

import constants as constants

robot = Robot()

samples = []

def start_routine(): 
    print(robot.hub.battery.voltage()/1000.0, "V")
    robot.hub.imu.reset_heading(0)

    #start_rover()
    #remove_ball()
    samples_routine()
    score_drone()
    green_white_routine()

    print(robot.hub.imu.heading())

def start_rover():
    dist = 170 # test on field; in mm

    robot.drive.straight_line_distance(180, 50)
    robot.drive.turn_angle(-90)
    robot.drive.straight_reflection_end(50)
    robot.drive.turn_angle(0)
    robot.drive.straight_line_distance(dist, 50)

    robot.arm.move_front_arm(160, wait=True)
    robot.drive.straight_distance(-dist + 20, 50)
    robot.arm.move_front_arm(80, wait=True)
    robot.drive.turn_angle(-90) #test if works
    robot.drive.straight_distance(-5, 50)
    robot.drive.turn_angle(180)
    
def remove_ball():
    robot.arm.move_front_arm(175, wait=True)
    robot.drive.straight_line_distance(150, 50)
    robot.drive.straight_distance(-100, 50)

def samples_routine(): 
    global samples 

    robot.arm.move_front_arm(90, wait=True)
    robot.drive.straight_reflection_start(50)
    robot.drive.straight_distance(150, 50)
    robot.drive.turn_angle(90)
    robot.drive.straight_time(500, -50)

    robot.hub.imu.reset_heading(0)
    samples = robot.drive.straight_scanner(1000, 50)
    print(samples)

def score_drone():
    robot.drive.straight_distance(-50, 50)
    robot.drive.turn_angle(90, wheel="left")
    robot.drive.straight_distance(800, 60)
    robot.drive.straight_distance(-1000, 60)
    robot.drive.turn_angle(180)
    robot.drive.straight_time(500, -50)

def deliver_samples_top(left):
    robot.hub.imu.reset_heading(0)
    robot.drive.straight_distance(-150, 50)

    robot.drive.turn_angle(90)
    robot.drive.straight_reflection_start(50)
    robot.drive.straight_line_distance(20, 50, "right")

    delivery_dist = 150

    #TODO: put samples down
    if (not left):
        robot.drive.turn_angle(-80)
    else:
        robot.drive.turn_angle(-110)


    robot.drive.straight_distance(-delivery_dist, 50)
    robot.arm.move_back_arm(180, wait=True)
    robot.drive.straight_distance(delivery_dist, 50)
    robot.drive.turn_angle(-90)

def green_white_routine():
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
    except ValueError:
        samples = [Colors.NONE] * 6
        green_pos = (5 - samples.index(Colors.NONE)) * between_mm + constants.kStartSamplesDistance
        white_pos = (5 - samples.index(Colors.NONE)) * between_mm + constants.kStartSamplesDistance

    diff = white_pos - green_pos

    samp_dist = 90 # test on field; in mm

    robot.drive.straight_distance(green_pos, 50)
    robot.drive.turn_angle(90)

    robot.arm.move_back_arm(165, wait=True)
    robot.drive.straight_distance(-samp_dist, 70)
    robot.arm.move_back_arm(100, wait=True)
    robot.drive.straight_distance(samp_dist, 70)

    robot.drive.turn_angle(0)
    robot.drive.straight_distance(diff, 70)
    
    #robot.drive.turn_angle(0)
    robot.drive.straight_time(2000, 60) #TODO: test on real field for imu reset

    deliver_samples_top(False)

    # field width
    field_width = 1150.5  # in mm

    robot.drive.straight_distance(450, 70)
    robot.drive.turn_angle(0)
    robot.drive.straight_time(1000, 70)
    robot.hub.imu.reset_heading(0)


    robot.drive.straight_distance(-(field_width-white_pos), 50)
    robot.drive.turn_angle(90)

    #
    robot.arm.move_back_arm(165, wait=True)
    robot.drive.straight_distance(-samp_dist, 70)
    robot.arm.move_back_arm(100, wait=True)
    robot.drive.straight_distance(samp_dist, 70)
    #

    robot.drive.turn_angle(0)
    robot.drive.straight_time(2500, 60) #TODO: test on real field for imu reset

    robot.hub.imu.reset_heading(0)
    deliver_samples_top(True)


    








