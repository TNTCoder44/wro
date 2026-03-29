###
### main.py - Entry point for the robot
###

from paths import *
from pybricks.tools import StopWatch

timer = StopWatch()

#start_routine()
test_samples()

if (timer.time()):
    print("Time taken: ", timer.time() / 1000.0, " seconds")
print("finished")