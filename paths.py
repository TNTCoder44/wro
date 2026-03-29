from robot import Robot

robot = Robot()

samples = []

def start_routine(): 
    robot.hub.imu.reset_heading(0)
    robot.drive.straight_time(750, -50)
    robot.hub.imu.reset_heading(0) ## reset gyro for starting at wall
    
    
def test_samples():
    robot.hub.imu.reset_heading(90)
    samples = robot.drive.straight_scanner(560, 70)
    print(samples)






