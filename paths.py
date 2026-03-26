from robot import Robot

robot = Robot()

def start_routine(): 
    robot.hub.imu.reset_heading(0)
    robot.drive.straight_time(750, -50)
    robot.hub.imu.reset_heading(0) ## reset gyro for starting at wall
    
    







