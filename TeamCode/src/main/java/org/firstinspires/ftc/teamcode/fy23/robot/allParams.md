Robot:
- double tpr                       | ticks per rotation
- double wheelDiameter             | in meters
- double maxForwardSpeed           | in meters per second
- PIDconsts hdgCorrectionPIDconsts | used by IMUcorrector
- clawParameters                   | subsystems are encapsulated in this class, so their parameters are too
- imuParameters                    | 
- driveParameters                  | 
- pixelArmParameters               | 
- planeLauncherParameters          | 

Claw:
- boolean present       | Is this subsystem installed on this robot?
- String clawServo      | The Servo object, already grabbed from the HardwareMap
- double openPosition   | The position (between 0 and 1) where the claw is considered "open" - determined experimentally
- double closedPosition | 

FriendlyIMU:
- boolean present | Is this subsystem installed on this robot?

MecanumDrive:
- boolean present                             | Is this subsystem installed on this robot?
- AccelLimiter accelLimiter                   | An AccelLimiter object, already instantiated
- DcMotorEx leftFrontMotor                    | The DcMotorEx object for the motor on the left front corner of the drivebase, already instantiated and configured (direction set)
- DcMotorEx rightFrontMotor                   | 
- DcMotorEx leftBackMotor                     | 
- DcMotorEx rightBackMotor                    | 
- DcMotor.RunMode runMode                     | The RunMode of all four motors
- DcMotor.ZeroPowerBehavior zeroPowerBehavior | The ZeroPowerBehavior of all four motors

PixelArm:
- boolean present                             | Is this subsystem installed on this robot?
- String pivotMotor                           | The pivot motor object, already grabbed from the HardwareMap (or pass in a MockDcMotorEx for testing)
- String elevatorMotor                        | The elevator motor object, already grabbed from the HardwareMap (or pass in a MockDcMotorEx for testing)
- AccelLimiter pivotAccelLimiter              | An AccelLimiter object that has been configured for the pivot motor
- PowerTpSConverter pivotPowerTpSConverter    | A PowerTpSConverter set up for the pivot motor
- double pivotTicksPerDegree                  | Encoder ticks traveled per degree of rotation
- int pivotUpperLimit                         | The upper limit of the pivot motor's motion, in encoder ticks
- int pivotLowerLimit                         | 
- DigitalDevice pivotUpperLimitSwitch         | A DigitalDevice that represents a physical upper limit switch (high limit in encoder ticks!) for the pivot motor
- DigitalDevice pivotLowerLimitSwitch         | 
- double maxPivotRecoveryPower                | The maximum power to use while the pivot arm is returning from a dangerous position
- AccelLimiter elevatorAccelLimiter           | An AccelLimiter object that has been configured for the elevator motor 
- PowerTpSConverter elevatorPowerTpSConverter | 
- double elevatorTicksPerMillimeter           | Encoder ticks (or fraction of a tick) traveled per millimeter of slide travel
- int elevatorUpperLimit                      | 
- int elevatorLowerLimit                      | 
- DigitalDevice elevatorUpperLimitSwitch      | 
- DigitalDevice elevatorLowerLimitSwitch      | 
- double maxElevatorRecoveryPower                | The maximum power to use while the elevator is returning from a dangerous position

PlaneLauncher:
- boolean present       | Is this subsystem installed on this robot?
- Servo planeServo      | The Servo object, already grabbed from the HardwareMap