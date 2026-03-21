###
### main.py - Entry point for the robot
###

from robot import Robot
from pybricks.tools import StopWatch

timer = StopWatch()

robot = Robot()

robot.drive.test_drive()

if (timer.time()):
    print("Time taken: ", timer.time() / 1000.0, " seconds")
print("finished")