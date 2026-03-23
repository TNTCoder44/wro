###
### DriveSubsystem class for the movement of the robot.
###

from pybricks.tools import wait, StopWatch
from pybricks.parameters import Stop

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

        self.straight_controller = PIDController(constants.kStraightPID)
        self.turn_controller = PIDController(constants.kTurnPID)
        self.line_controller = PIDController(constants.kLinePID)

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

    def straight_time(self, time_ms, power, target_angle = -1):
        self.reset_encoder_position()
        self.straight_controller.reset()

        if target_angle == -1:
            target_angle = self.hub.imu.heading() # use current heading as targetangle
        
        time_of_driving = StopWatch()

        while time_of_driving.time() < time_ms:
            current_heading = self.hub.imu.heading()

            error = target_angle - current_heading
            correction = self.straight_controller.calculate(error)

            # no trapezoidal motion profile because we don't know the distance
            # using constant power instead
            leftPower = power + correction
            rightPower = power - correction

            self.left.dc(leftPower)
            self.right.dc(rightPower)

        self.stop_for_time(10) # brake for 10 milliseconds to ensure a complete stop

    # drive until black line detected
    def straight_reflection_start(self, reflection_color, power, sensor_in_use='line'):
        self.reset_encoder_position()
        self.straight_controller.reset()

        if sensor_in_use == 'color':
            sensor = self.color_sensor
        else:
            sensor = self.line_sensor # default choice
            
        target_angle = self.hub.imu.heading() # use current heading as targetangle

        while True:
            current_heading = self.hub.imu.heading()
            current_reflection = sensor.reflection()


            error = target_angle - current_heading
            correction = self.straight_controller.calculate(error)

            self.left.dc(power + correction)
            self.right.dc(power - correction)

            # if a dark color is detected, we need to check if reflection is below threshold
            if reflection_color < 50 and sensor_in_use == 'line': 
                if current_reflection < reflection_color:
                    break
            else: 
                if (current_reflection > reflection_color):
                    break
                # if a light color is detected, we need to check if reflection is above threshold
                # this also applies to the color sensor, because we only want to detect bright colors

        self.stop_for_time(10) # brake for 10 milliseconds to ensure a complete stop

    def straight_reflection_end(self):
        # drive straight until line is finished
        pass

    def straight_line_distance(self, distance_mm, power, side='left'):
        self.reset_encoder_position()
        self.line_controller.reset()

        # target reflection value for line following
        # TODO: test on robot with actual line reflection
        target_reflection = constants.kReflectionBlack + constants.kReflectionWhite / 2
        
        target_degrees = abs(self.get_degrees_from_mm(distance_mm))

        direction = 1 if side == 'left' else -1

        while abs(self.get_motor_position()) < target_degrees:
            current_reflection = self.line_sensor.reflection()

            # pid controller for heading correction
            error = target_reflection - current_reflection
            correction = self.line_controller.calculate(error)

            left_power = power + correction * direction
            right_power = power - correction * direction

            max_power = max(abs(left_power), abs(right_power), 100) # maximum 100%
            left_power = (left_power / max_power) * 100
            right_power = (right_power / max_power) * 100

            self.left.dc(left_power)
            self.right.dc(right_power)

        self.stop_for_time(10) # brake for 10 milliseconds to ensure a complete stop

    def turn_angle(self, target_angle, max_power=75, exit_time=1500):
        # reset pid controller
        self.turn_controller.reset()
        
        # because of the use of a while true loop, we need an exit condition
        # to prevent infinite looping in case of sensor/pid failure 
        exit_timer = StopWatch()

        while True:
            current_angle = self.hub.imu.heading()

            ## clamp error to -180 to 180 range, 
            ## because error can be maximum 180 degrees in either direction from current heading
            ## -> method should be efficient 
            error = target_angle - current_angle 
            error = (error + 180) % 360 - 180

            correction = self.turn_controller.calculate(error)
            
            # apply correction directly to motors therefore clamp to max power
            correction = max(-max_power, min(correction, max_power))

            # exit conditions
            if abs(error) < constants.kErrorForTurn: # if within error threshold, stop
                break

            if exit_timer.time() > exit_time: # if taking too long, exit to prevent infinite loop
                break

            # minimum power
            if abs(correction) < constants.kMinimumPower and abs(error) > constants.kErrorForTurn:
                correction = constants.kMinimumPower * (1 if correction > 0 else -1)

            self.left.dc(correction)
            self.right.dc(-correction)

        self.stop_for_time(5)

    # fallback methods for using each drive motor individually
    def left_degrees(self, degrees, speed, then=Stop.HOLD, wait=True):
        self.left.run_angle(speed, degrees, then=Stop.HOLD, wait=wait)

    def right_degrees(self, degrees, speed, then=Stop.HOLD, wait=True):
        self.right.run_angle(speed, degrees, then=Stop.HOLD, wait=wait)

    def left_time(self, time_ms, speed, then=Stop.HOLD, wait=True):
        self.left.run_time(speed, time_ms, then=Stop.HOLD, wait=wait)

    def right_time(self, time_ms, speed, then=Stop.HOLD, wait=True):
        self.right.run_time(speed, time_ms, then=Stop.HOLD, wait=wait)
