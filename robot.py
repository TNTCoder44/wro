from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Direction

import utils.constants as constants

from subsystems.drive import DriveSubsystem
from subsystems.arm import ArmSubsystem

class Robot: 
    def __init__(self):
        self.hub = PrimeHub()

        self.left_drive_motor = Motor(constants.kPortLeftDriveMotor, Direction.COUNTERCLOCKWISE)
        self.right_drive_motor = Motor(constants.kPortRightDriveMotor, Direction.CLOCKWISE)
        
        self.front_motor_arm = Motor(constants.kPortArmFrontMotor, Direction.CLOCKWISE)
        self.back_motor_arm = Motor(constants.kPortArmBackMotor, Direction.CLOCKWISE)

        self.color_sensor = ColorSensor(constants.kPortColorSensor)
        self.line_sensor = ColorSensor(constants.kPortLineSensor)

        self.drive = DriveSubsystem(
            self.hub,
            self.left_drive_motor,
            self.right_drive_motor,
            self.color_sensor,
            self.line_sensor
        )

        self.arm = ArmSubsystem(
            self.front_motor_arm,
            self.back_motor_arm
        )

