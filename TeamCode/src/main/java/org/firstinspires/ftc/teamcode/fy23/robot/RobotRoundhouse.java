package org.firstinspires.ftc.teamcode.fy23.robot;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.BlankMotor;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.digitaldevice.DigitalDeviceBlank;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.FriendlyIMUImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PixelArmImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PlaneLauncherImpl;
import org.firstinspires.ftc.teamcode.fy23.units.PIDconsts;
import org.firstinspires.ftc.teamcode.fy23.units.SimplePowerTpSConverter;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class RobotRoundhouse {

    public static Robot.Parameters getRobotAParams(HardwareMap hardwareMap) {
        Claw.Parameters clawParams = new Claw.Parameters();
        clawParams.present = true;
        clawParams.clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawParams.openPosition = 0.1;
        clawParams.closePosition = 0.01;

        FriendlyIMUImpl.Parameters imuParams = new FriendlyIMUImpl.Parameters();
        imuParams.present = true;

        MecanumDriveImpl.Parameters driveParams = new MecanumDriveImpl.Parameters();
        driveParams.present = true;

        driveParams.accelLimiter = new AccelLimiter(2.0, 0.1);

        driveParams.leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        driveParams.leftFrontMotor.setDirection(REVERSE);

        driveParams.rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        driveParams.rightFrontMotor.setDirection(FORWARD);

        driveParams.leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        driveParams.leftBackMotor.setDirection(REVERSE);

        driveParams.rightBackMotor = hardwareMap.get(DcMotorEx.class, "armPivot");
        driveParams.rightBackMotor.setDirection(FORWARD);

        driveParams.runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        driveParams.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

        PixelArmImpl.Parameters armParams = new PixelArmImpl.Parameters();
        armParams.present = true;
        armParams.pivotMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
        armParams.elevatorMotor = hardwareMap.get(DcMotorEx.class, "armExtend");
        armParams.pivotAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
        armParams.pivotPowerTpSConverter = new SimplePowerTpSConverter(6472, 12949); // TODO: not measured on real hardware!!
        armParams.pivotTicksPerDegree = 10; // TODO: not measured!!
        armParams.pivotUpperLimit = 2000; // TODO: not measured on real hardware!!
        armParams.pivotLowerLimit = 0; // TODO: not measured on real hardware!!
        armParams.pivotUpperLimitSwitch = new DigitalDeviceBlank(); // not installed
        armParams.pivotLowerLimitSwitch = new DigitalDeviceBlank(); // not installed
        armParams.maxPivotRecoveryPower = 0.2;
        armParams.elevatorAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
        armParams.elevatorPowerTpSConverter = new SimplePowerTpSConverter(1249, 2499); // TODO: not measured on real hardware!!
        armParams.elevatorTicksPerMillimeter = 10; // TODO: not measured!!
        armParams.elevatorUpperLimit = 2500;
        armParams.elevatorLowerLimit = 0;
        armParams.elevatorUpperLimitSwitch = new DigitalDeviceBlank(); // not installed
        armParams.elevatorLowerLimitSwitch = new DigitalDeviceBlank(); // not installed
        armParams.maxElevatorRecoveryPower = 0.2;

        PlaneLauncherImpl.Parameters planeLauncherParams = new PlaneLauncherImpl.Parameters();
        planeLauncherParams.present = true;
        planeLauncherParams.planeServo = hardwareMap.get(Servo.class,"planeservo");

        Robot.Parameters params = new Robot.Parameters();
        params.tpr = 537.7; // ticks per rotation
        params.wheelDiameter = 0.096; // in meters
        params.maxForwardSpeed = 1.50; // in meters per second
        params.hdgCorrectionPIDconsts = new PIDconsts(0.023, 0, 0);

        params.clawParameters = clawParams;
        params.imuParameters = imuParams;
        params.driveParameters = driveParams;
        params.pixelArmParameters = armParams;
        params.planeLauncherParameters = planeLauncherParams;

        return params;
    }

    public static Robot.Parameters getRobotBParams(HardwareMap hardwareMap) {
        Claw.Parameters clawParams = new Claw.Parameters();
        clawParams.present = false;

        FriendlyIMUImpl.Parameters imuParams = new FriendlyIMUImpl.Parameters();
        imuParams.present = true;

        MecanumDriveImpl.Parameters driveParams = new MecanumDriveImpl.Parameters();
        driveParams.present = true;

        driveParams.accelLimiter = new AccelLimiter(2.0, 0.1);

        driveParams.leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        driveParams.leftFrontMotor.setDirection(REVERSE);

        driveParams.rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        driveParams.rightFrontMotor.setDirection(FORWARD);

        driveParams.leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        driveParams.leftBackMotor.setDirection(REVERSE);

        driveParams.rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
        driveParams.rightBackMotor.setDirection(FORWARD);

        driveParams.runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        driveParams.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

        PixelArmImpl.Parameters armParams = new PixelArmImpl.Parameters();
        armParams.present = false;

        PlaneLauncherImpl.Parameters planeLauncherParams = new PlaneLauncherImpl.Parameters();
        planeLauncherParams.present = false;

        Robot.Parameters params = new Robot.Parameters();
        params.tpr = 537.7;
        params.wheelDiameter = 0.096; // in meters
        params.maxForwardSpeed = 1.50; // in meters per second
        params.hdgCorrectionPIDconsts = new PIDconsts(0.023, 0, 0);

        params.clawParameters = clawParams;
        params.imuParameters = imuParams;
        params.driveParameters = driveParams;
        params.pixelArmParameters = armParams;
        params.planeLauncherParameters = planeLauncherParams;

        return params;
    }

    public static Robot.Parameters getVirtualRobotParams(HardwareMap hardwareMap) {
        Claw.Parameters clawParams = new Claw.Parameters();
        clawParams.present = false;

        FriendlyIMUImpl.Parameters imuParams = new FriendlyIMUImpl.Parameters();
        imuParams.present = true;

        MecanumDriveImpl.Parameters driveParams = new MecanumDriveImpl.Parameters();
        driveParams.present = true;

        driveParams.accelLimiter = new AccelLimiter(2.0, 0.1);

        driveParams.leftFrontMotor = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        driveParams.leftFrontMotor.setDirection(REVERSE);

        driveParams.rightFrontMotor = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        driveParams.rightFrontMotor.setDirection(FORWARD);

        driveParams.leftBackMotor = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        driveParams.leftBackMotor.setDirection(REVERSE);

        driveParams.rightBackMotor = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        driveParams.rightBackMotor.setDirection(FORWARD);

        driveParams.runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        driveParams.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

        PixelArmImpl.Parameters armParams = new PixelArmImpl.Parameters();
        armParams.present = false;

        PlaneLauncherImpl.Parameters planeLauncherParams = new PlaneLauncherImpl.Parameters();
        planeLauncherParams.present = false;

        Robot.Parameters params = new Robot.Parameters();
        params.tpr = 537.7;
        params.wheelDiameter = 0.096; // in meters
        params.maxForwardSpeed = 1.50; // in meters per second
        params.hdgCorrectionPIDconsts = new PIDconsts(0.023, 0, 0);

        params.clawParameters = clawParams;
        params.imuParameters = imuParams;
        params.driveParameters = driveParams;
        params.pixelArmParameters = armParams;
        params.planeLauncherParameters = planeLauncherParams;

        return params;
    }

    public static Robot.Parameters getProgrammingBoardParams(HardwareMap hardwareMap) {
        Claw.Parameters clawParams = new Claw.Parameters();
        clawParams.present = false;
        clawParams.clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawParams.openPosition = 0.1;
        clawParams.closePosition = 0.01;

        FriendlyIMUImpl.Parameters imuParams = new FriendlyIMUImpl.Parameters();
        imuParams.present = false;

        MecanumDriveImpl.Parameters driveParams = new MecanumDriveImpl.Parameters();
        driveParams.present = false;

        driveParams.accelLimiter = new AccelLimiter(2.0, 0.1);

        driveParams.leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        driveParams.leftFrontMotor.setDirection(REVERSE);

        driveParams.rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        driveParams.rightFrontMotor.setDirection(FORWARD);

        driveParams.leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        driveParams.leftBackMotor.setDirection(REVERSE);

        driveParams.rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
        driveParams.rightBackMotor.setDirection(FORWARD);

        driveParams.runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        driveParams.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

        PixelArmImpl.Parameters armParams = new PixelArmImpl.Parameters();
        armParams.present = true;
        armParams.pivotMotor = hardwareMap.get(DcMotorEx.class, "motor");
//        armParams.elevatorMotor = hardwareMap.get(DcMotorEx.class, "armExtend");
        armParams.elevatorMotor = new BlankMotor();
        armParams.pivotAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
        armParams.pivotPowerTpSConverter = new SimplePowerTpSConverter(6472, 12949); // TODO: not measured on real hardware!!
        armParams.pivotTicksPerDegree = 10; // TODO: not measured!!
        armParams.pivotUpperLimit = 2000; // TODO: not measured on real hardware!!
        armParams.pivotLowerLimit = 0; // TODO: not measured on real hardware!!
        armParams.pivotUpperLimitSwitch = new DigitalDeviceBlank(); // not installed
        armParams.pivotLowerLimitSwitch = new DigitalDeviceBlank(); // not installed
        armParams.maxPivotRecoveryPower = 0.2;
        armParams.elevatorAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
        armParams.elevatorPowerTpSConverter = new SimplePowerTpSConverter(1249, 2499); // TODO: not measured on real hardware!!
        armParams.elevatorTicksPerMillimeter = 10; // TODO: not measured!!
        armParams.elevatorUpperLimit = 2500;
        armParams.elevatorLowerLimit = 0;
        armParams.elevatorUpperLimitSwitch = new DigitalDeviceBlank(); // not installed
        armParams.elevatorLowerLimitSwitch = new DigitalDeviceBlank(); // not installed
        armParams.maxElevatorRecoveryPower = 0.2;

        PlaneLauncherImpl.Parameters planeLauncherParams = new PlaneLauncherImpl.Parameters();
        planeLauncherParams.present = false;
        planeLauncherParams.planeServo = hardwareMap.get(Servo.class,"planeServo");

        Robot.Parameters params = new Robot.Parameters();
        params.tpr = 537.7; // ticks per rotation
        params.wheelDiameter = 0.096; // in meters
        params.maxForwardSpeed = 1.50; // in meters per second
        params.hdgCorrectionPIDconsts = new PIDconsts(0.023, 0, 0);

        params.clawParameters = clawParams;
        params.imuParameters = imuParams;
        params.driveParameters = driveParams;
        params.pixelArmParameters = armParams;
        params.planeLauncherParameters = planeLauncherParams;

        return params;
    }

}
