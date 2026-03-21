###
### ArmSubsystem class for controlling the arm mechanism
###

class ArmSubsystem:
    def __init__(self, front_motor, back_motor):
        self.front = front_motor
        self.front.reset_angle(0)
        self.back = back_motor
        self.back.reset_angle(0)

    def move_arm(self, position: float):
        # Implement the logic to control the arm based on the desired position
        pass