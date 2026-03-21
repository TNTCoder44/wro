###
### Constants used for the project
###

from pybricks.parameters import Port

PI = 3.14159

kPortLeftDriveMotor = Port.A
kPortRightDriveMotor = Port.B

kPortArmBackMotor = Port.C
kPortArmFrontMotor = Port.D

kPortColorSensor = Port.E
kPortLineSensor = Port.F

kWheelDiameter = 50 ## in millimeters
kDegsPerMM = 360 / (PI * kWheelDiameter) ## degrees per millimeter

kStraightPID = (2.0, 0.0, 0.3) ## PID for straight driving: (kP, kI, kD)

kMinimumPower = 10
