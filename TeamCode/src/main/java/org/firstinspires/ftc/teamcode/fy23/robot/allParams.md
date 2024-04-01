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
- String clawServoName  | The name of the servo in the HardwareMap
- double openPosition   | The position (between 0 and 1) where the claw is considered "open" - determined experimentally
- double closedPosition | 

FriendlyIMU:
- boolean present | Is this subsystem installed on this robot?

MecanumDrive:
- boolean present                             | Is this subsystem installed on this robot?
- double maxMotorAccel                        | Maximum acceleration, in power per second per second
- double maxDeltaVEachLoop                    | Maximum change in velocity each loop - prevents jerking
- String leftFrontName                        | Name of the left front motor in the HardwareMap
- DcMotor.Direction leftFrontDirection        | The direction the left front motor spins when positive power is applied (FORWARD being clockwise)
- String rightFrontName                       | 
- DcMotor.Direction rightFrontDirection       | 
- String leftBackName                         | 
- DcMotor.Direction leftBackDirection         | 
- String rightBackName                        | 
- DcMotor.Direction rightBackDirection        | 
- DcMotor.RunMode runMode                     | The RunMode of all four motors
- DcMotor.ZeroPowerBehavior zeroPowerBehavior | The ZeroPowerBehavior of all four motors

PixelArm:
- boolean present                           | Is this subsystem installed on this robot?
- String pivotMotorName                     | The name of the pivot motor in the HardwareMap
- String elevatorMotorName                  | The name of the elevator motor in the HardwareMap
- double maxPivotAccel                      | The maximum acceleration for the pivot motor
- double maxPivotDeltaVEachLoop             | The maximum change in velocity each loop for the pivot motor (prevents jerking)
- int pivotUpperLimit                       | The upper limit of the pivot motor's motion, in encoder ticks
- int pivotLowerLimit                       |
- double maxElevatorAccel                   | 
- double maxElevatorDeltaVEachLoop          | 
- int elevatorUpperLimit                    | 
- int elevatorLowerLimit                    |

PlaneLauncher:
- boolean present       | Is this subsystem installed on this robot?
- String planeServoName | The name of the servo in the HardwareMap