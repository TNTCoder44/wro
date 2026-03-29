###
### Constants used for the project
###

from pybricks.parameters import Port, Direction

PI = 3.14159

kPortLeftDriveMotor = Port.A
kDirectionLeftDrive = Direction.COUNTERCLOCKWISE
kPortRightDriveMotor = Port.B
kDirectionRightDrive = Direction.CLOCKWISE


kPortArmBackMotor = Port.C
kDirectionArmBack = Direction.CLOCKWISE
kPortArmFrontMotor = Port.D
kDirectionArmFront = Direction.CLOCKWISE

kPortColorSensor = Port.E
kPortLineSensor = Port.F

kWheelDiameter = 50 ## in millimeters
kDegsPerMM = 360 / (PI * kWheelDiameter) ## degrees per millimeter

kStartScanningDegrees = 340 # TODO: test on real field to determine startpos
kDegreesBetweenSamples = 175 #test

kErrorForTurn = 1.0

# reflection values for line sensor used for line following
kReflectionBlack = 15
kReflectionWhite = 75
kReflectionAvg = 60

kReflectionError = 20

kStraightPID = (11.0, 0.0, 0.1) ## PID for straight driving: (kP, kI, kD)
kTurnPID = (5.5, 0.0, 0.25) ## PID for turning: (kP, kI, kD)
kLinePID = (0.5, 0.0, 0.2) ## PID for line following: (kP, kI, kD)

kMinimumPower = 10

kSpeedArm = 360
