###
### ArmSubsystem class for controlling the arm mechanism
###

from pybricks.parameters import Stop
import constants as constants
class ArmSubsystem:
    def __init__(self, front_motor, back_motor):
        self.front = front_motor
        self.front.reset_angle(0)
        self.back = back_motor
        #self.back.reset_angle(0)
        self.speed = constants.kSpeedArm

    # give position in degrees
    # no need for pid control because standard contorl will be accurate enough for simple motion
    def move_front_arm(self, position: float, wait=True):
       self.front.run_target(self.speed, position, then=Stop.HOLD, wait=wait) 

    def move_back_arm(self, position: float, wait=True):
        self.back.run_target(self.speed, position, then=Stop.HOLD, wait=wait)
    
