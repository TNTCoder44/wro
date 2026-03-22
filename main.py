###
### main.py - Entry point for the robot
###

from paths import *
from pybricks.tools import StopWatch

timer = StopWatch()

start_routine()

if (timer.time()):
    print("Time taken: ", timer.time() / 1000.0, " seconds")
print("finished")