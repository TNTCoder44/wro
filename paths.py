from robot import Robot
from pybricks.tools import wait

from scanner import Colors

robot = Robot()

samples = []

def start_routine(): 
    print(robot.hub.battery.voltage()/1000.0, "V")
    robot.hub.imu.reset_heading(0)
    #robot.drive.straight_time(750, -40)
    #robot.hub.imu.reset_heading(0) ## reset gyro for starting at wall

    #robot.drive.straight_reflection_end(100)

    robot.drive.turn_angle(90, wheel="left")
    robot.drive.turn_angle(180, wheel="right")
    
    robot.hub.speaker.beep(200, 100)
       
    return

    #robot.drive.straight_distance(700, 70)
    #robot.drive.straight_reflection_end(100)
    robot.drive.straight_distance(800, 75)
    robot.hub.speaker.beep(200, 100)
    robot.drive.turn_angle(180)
    samples = robot.drive.straight_scanner(700, 75)
    #samples = robot.drive.straight_scanner(700, 75)


    wait(1000)

    print(samples)
    
    
def test_samples():
    robot.hub.imu.reset_heading(90)
    samples = robot.drive.straight_scanner(560, 70)
    print(samples)






