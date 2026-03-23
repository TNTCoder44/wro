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

kErrorForTurn = 1.0

# reflection values for line sensor used for line following
kReflectionBlack = 15
kReflectionWhite = 75

kStraightPID = (11.0, 0.0, 0.1) ## PID for straight driving: (kP, kI, kD)
kTurnPID = (5.5, 0.0, 0.25) ## PID for turning: (kP, kI, kD)
kLinePID = (0.5, 0.0, 0.2) ## PID for line following: (kP, kI, kD)

kMinimumPower = 10
