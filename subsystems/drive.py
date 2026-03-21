###
### DriveSubsystem class for the movement of the robot.
###

from pybricks.tools import wait, StopWatch

from subsystems.scanner import ScannerSubsystem
from utils.pid_controller import PIDController
import utils.constants as constants

class DriveSubsystem:
    def __init__(self, hub, left_motor, right_motor, color_sensor, line_sensor):
        self.hub = hub

        self.left = left_motor
        self.left.reset_angle(0)

        self.right = right_motor
        self.right.reset_angle(0)

        self.color_sensor = color_sensor
        self.line_sensor = line_sensor

        self.straight_controller = PIDController(constants.kStraightPID, max_output=30)

        self.scanner = ScannerSubsystem(
            self.color_sensor
        )

    def test_drive(self):
        errors = [0.1, 0.3, 0.05, 0.01, 0.5, -1, 0]

        for e in errors:
            print(self.straight_controller.calculate(e))

    # get motor encoder position in degrees
    def get_motor_position(self):
        return abs(self.left.angle() + self.right.angle()) / 2
    
    # as an integer, easier to work with motors
    def get_degrees_from_mm(self, distance_mm):
        return int(distance_mm * constants.kDegsPerMM)
    
    # set absolute encoder position
    def reset_encoder_position(self):
        self.left.reset_angle(0)
        self.right.reset_angle(0)

    # stop driving for an amount of time, in milliseconds
    def stop_for_time(self, time_ms):
        self.left.brake()
        self.right.brake()
        wait(time_ms) ## stop thread for time_ms milliseconds


    # drive straight for a certain distance 
    # use trapezoidal motion profile for smooth drive path
    # use pid controller for heading correction
    # use duty cycle for trapezoidal motion because it is more versatile for custom motion control
    def straight_distance(self, distance_mm, max_power, target_angle = -1, 
                                trapezoidal_speed = True, accel_ratio = 0.3, 
                                accel = True, decel = True):
        # reset angles to start from 0
        self.reset_encoder_position()
        self.straight_controller.reset()

        # dutycycle is between 0 and 100
        max_power = max(0, min(max_power, 100))

        if (target_angle == -1):
            target_angle = self.hub.imu.heading() # use current heading as targetangle

        target_degrees = self.get_degrees_from_mm(distance_mm)
        direction = 1 if distance_mm >= 0 else -1

        accel_dist = target_degrees * accel_ratio
        decel_dist = target_degrees * accel_ratio if direction == 1 else target_degrees * accel_ratio * 2
        relative_distance = target_degrees ## distane left

        while relative_distance >= 0:
            current_heading = self.hub.imu.heading()
            current_degrees = self.get_motor_position()
            relative_distance = target_degrees - current_degrees

            # heading pid controller
            error = target_angle - current_heading
            correction = self.straight_controller.calculate(error)

            # trapezoidal motion profile
            if trapezoidal_speed and distance_mm != 0:
                if accel and current_degrees < accel_dist:
                    power = max_power * (current_degrees / accel_dist)
                elif decel and relative_distance < decel_dist:
                    power = max_power * (relative_distance / decel_dist)
                else:
                    power = max_power

                power = max(constants.kMinimumPower, min(power, max_power)) * direction

                leftPower = power + correction
                rightPower = power - correction

                self.left.dc(leftPower)
                self.right.dc(rightPower)
        
            else:
                leftPower = max_power * direction + correction
                rightPower = max_power * direction - correction

                self.left.dc(leftPower)
                self.right.dc(rightPower)

        self.stop_for_time(10) # brake for 10 milliseconds to ensure a complete stop


    def turn_to_angle(self, target_angle, max_power):
        pass

