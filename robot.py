from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor

import constants as constants

from drive import DriveSubsystem
from arm import ArmSubsystem

class Robot: 
    def __init__(self):
        self.hub = PrimeHub()

        self.left_drive_motor = Motor(constants.kPortLeftDriveMotor, constants.kDirectionLeftDrive)
        self.right_drive_motor = Motor(constants.kPortRightDriveMotor, constants.kDirectionRightDrive)
        
        #self.front_motor_arm = Motor(constants.kPortArmFrontMotor, constants.kDirectionArmFront)
        #self.back_motor_arm = Motor(constants.kPortArmBackMotor, constants.kDirectionArmBack)

        self.color_sensor = ColorSensor(constants.kPortColorSensor)
        self.line_sensor = ColorSensor(constants.kPortLineSensor)

        self.drive = DriveSubsystem(
            self.hub,
            self.left_drive_motor,
            self.right_drive_motor,
            self.color_sensor,
            self.line_sensor
        )
        return
        self.arm = ArmSubsystem(
            self.front_motor_arm,
            self.back_motor_arm
        )

