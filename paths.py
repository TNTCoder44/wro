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
    #samples_routine()
    #score_drone()
    #green_white_routine()

    robot.arm.move_back_arm(140, wait=True)

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
    robot.drive.straight_distance(-900, 60)
    robot.drive.turn_angle(180)
    robot.drive.straight_time(500, -50)

def green_white_routine():
    global samples
    
    robot.hub.imu.reset_heading(0)
    # reverse samples, because we start from the other side of the field
    samp = samples[::-1]
    
    # go until green sample position, then back/forward until white sample
    # finally go against wall at the top to reset imu

    between_mm = constants.kDistanceBetweenSamples * constants.kDegsPerMM

    #.index sometimes throws an error; this cannot be used
    try:
        green_pos = samp.index(Colors.GREEN) * between_mm + constants.kStartSamplesDistance
        white_pos = samp.index(Colors.WHITE) * between_mm + constants.kStartSamplesDistance
    except ValueError:
        green_pos = samp.index(Colors.NONE) * between_mm + constants.kStartSamplesDistance
        white_pos = samp.index(Colors.NONE) * between_mm + constants.kStartSamplesDistance

    diff = white_pos - green_pos

    samp_dist = 50 # test on field; in mm

    robot.drive.straight_distance(green_pos, 50)
    robot.drive.turn_angle(90)

    robot.arm.move_back_arm(140, wait=True)
    robot.drive.straight_distance(-samp_dist, 70)
    robot.arm.move_back_arm(100, wait=True)
    robot.drive.straight_distance(samp_dist, 70)

    robot.drive.turn_angle(0)
    robot.drive.straight_distance(diff, 70)
    robot.drive.turn_angle(90)

    robot.arm.move_front_arm(140, wait=True)
    robot.drive.straight_distance(-samp_dist, 70)
    robot.arm.move_front_arm(100, wait=True)
    robot.drive.straight_distance(samp_dist, 70)

    robot.drive.turn_angle(0)
    robot.drive.straight_time(1500, 60) #TODO: test on real field for imu reset

    robot.hub.imu.reset_heading(0)
    robot.drive.straight_distance(-150, 50)
    robot.drive.turn_angle(90)
    robot.drive.straight_reflection_start(50)
    #TODO: put samples down

    








