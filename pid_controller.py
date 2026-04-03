###
### PID Controller class for error correction and motion control
###

from pybricks.tools import StopWatch

class PIDController:
    def __init__(self, pid_values, i_zone = None, min_output=None, max_output=None):
        self.kP = pid_values[0]
        self.kI = pid_values[1]
        self.kD = pid_values[2]
        
        self.integral = 0

        self.integral_zone = i_zone
        self.previous_error = 0
        self.timer = StopWatch()

        self.min_output = min_output
        self.max_output = max_output

    def reset(self):
        self.integral = 0
        self.previous_error = 0
        self.timer.reset()

    def calculate(self, error):
        p = self.kP * error

        dt = self.timer.time() / 1000.0 # convert to seconds
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        d = self.kD * derivative

        if self.integral_zone is None or abs(error) <= self.integral_zone:
            self.integral += error * dt
        else: 
            self.integral = 0 # reset integral if error is outside of integral zone to prevent windup
        
        i = self.kI * self.integral

        output = p + d + i

        self.timer.reset()
        self.previous_error = error

        # apply min and max output limits
        if self.min_output is not None and abs(output) > 0 and abs(output) < self.min_output:
            output = self.min_output * (1 if output > 0 else -1)

        if self.max_output is not None and abs(output) > self.max_output:
            output = self.max_output * (1 if output > 0 else -1)



        return output
    