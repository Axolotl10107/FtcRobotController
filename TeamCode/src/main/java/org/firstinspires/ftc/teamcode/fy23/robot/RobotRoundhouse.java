package org.firstinspires.ftc.teamcode.fy23.robot;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.network.ControlHubDeviceNameManager;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.*;
import org.firstinspires.ftc.teamcode.fy23.units.PIDConsts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class RobotRoundhouse {

    public static Robot24.Parameters getParamsAuto(String serialNumber, HardwareMap hardwareMap) {
        switch (serialNumber) {
            case "10107-A-RC":
                return getRobotAParams(hardwareMap);
            case "10107-B-RC":
            case "10107-B":
                return getRobotBParams(hardwareMap);
            default:
                return getRobotAParams(hardwareMap);
        }
    }

    public static Robot24.Parameters getParamsAuto(HardwareMap hardwareMap) {
        // ControlHubDeviceNameManager doesn't exist in virtual_robot, so if you're using that, comment this out in your
        // virtual_robot project and have this return the virtualRobot or programmingBoard parameters.
        return getParamsAuto(ControlHubDeviceNameManager.getControlHubDeviceNameManager().getDeviceName(), hardwareMap);
    }

    public static Robot24.Parameters getRobotAParams(HardwareMap hardwareMap) {

        Claw.Parameters clawParams = new Claw.Parameters(false, 0.1, 0.01);
//        clawParams.clawServo = hardwareMap.get(Servo.class, "clawServo");

        RotaryIntake.Parameters intakeParams = new RotaryIntake.Parameters(false);

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters(true, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        RRMecanumDrive.DriveConstants dc = new RRMecanumDrive.DriveConstants();

        dc.TICKS_PER_REV = 537.7;
        dc.MAX_RPM = 117;
        dc.RUN_USING_ENCODER = true;
        dc.MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
                dc.getMotorVelocityF(dc.MAX_RPM / 60 * dc.TICKS_PER_REV));

        dc.WHEEL_RADIUS = 1.88976;
        dc.GEAR_RATIO = 1;
        dc.TRACK_WIDTH = 15.9;

        dc.kV = .017;
        dc.kA = .002;
        dc.kStatic = 0.0001;

        dc.MAX_ANG_VEL = 3;
        dc.MAX_ANG_ACCEL = 3;

        dc.MAX_VEL = 30;
        dc.MAX_ACCEL = 30;

        RRMecanumDrive.Parameters driveParams = new RRMecanumDrive.Parameters(true,
                dc,
                new AccelLimiter(2.0, 0.1));

        driveParams.TRANSLATIONAL_PID = new PIDCoefficients(4, 0, 1);
        driveParams.HEADING_PID = new PIDCoefficients(4, 0, 0);
        driveParams.LATERAL_MULTIPLIER = 1.2;
        driveParams.VX_WEIGHT = 1;
        driveParams.VY_WEIGHT = 1;
        driveParams.OMEGA_WEIGHT = 1;


        driveParams.useAccelLimiter = true;

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

        driveParams.stopwatch = new ElapsedTime();

        DoubleArm.Parameters armParams = new DoubleArm.Parameters(false);

        Robot24.ExtendedParameters extendedParams = new Robot24.ExtendedParameters();
        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0.023, 0, 0,0 );

        Robot24.Parameters params = new Robot24.Parameters(clawParams, intakeParams, imuParams, driveParams, armParams, /*planeLauncherParams,*/ extendedParams);
//        params.tpr = 537.7; // ticks per rotation
//        params.wheelDiameter = 0.096; // in meters
//        params.maxForwardSpeed = 1.50; // in meters per second

        return params;
    }

// ---------------------------------------------------------------------------------------------------------------------

    public static Robot24.Parameters getRobotBParams(HardwareMap hardwareMap) {

        Claw.Parameters clawParams = new Claw.Parameters(true, 0.1, 0.01);
        RotaryIntake.Parameters intakeParams = new RotaryIntake.Parameters(true);

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters(true, RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);

        RRMecanumDrive.DriveConstants dc = new RRMecanumDrive.DriveConstants();
        dc.LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        dc.USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RRMecanumDrive.Parameters driveParams = new RRMecanumDrive.Parameters(true,
                dc, // most of the defaults in DriveConstants should work here
                new AccelLimiter(2.0, 0.1));
        // TODO: make RRMecanumDrive use velocity instead of power
        driveParams.useAccelLimiter = true;

        dc.kV = .017;
        dc.kA = .002;
        dc.kStatic = 0.0001;

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

        driveParams.stopwatch = new ElapsedTime();

        // the rest of the defaults in RRMecanumDrive.Parameters should work here

        DoubleArm.Parameters armParams = new DoubleArm.Parameters(true);

        armParams.stopwatch = new ElapsedTime();

        armParams.elevatorMotorLeft = hardwareMap.get(DcMotorEx.class, "armLeftExtend");
        armParams.elevatorMotorRight = hardwareMap.get(DcMotorEx.class, "armRightExtend");

        armParams.elevatorMotorLeft.setDirection(FORWARD);
        armParams.elevatorMotorRight.setDirection(REVERSE);

        armParams.pivotMotorLeft = hardwareMap.get(DcMotorEx.class, "armLeftPivot");
        armParams.pivotMotorRight = hardwareMap.get(DcMotorEx.class, "armRightPivot");

        armParams.pivotMotorLeft.setDirection(REVERSE);
        armParams.pivotMotorRight.setDirection(FORWARD);

        // Due to a bug in AccelLimiter (I think?), these values should be 10 times higher than
        // what you actually want.
        armParams.pivotAccelLimiter = new AccelLimiter(4000, 4000);
        armParams.elevatorAccelLimiter = new AccelLimiter(4000, 4000);

        armParams.pivotUpperLimit = 1000;
        armParams.elevatorUpperLimit = 42;
        armParams.elevatorLimitBuffer = 2;
        armParams.elevatorOffsetLength = 3.5;

        armParams.maxPivotVelocity = 400;
        armParams.maxElevatorVelocity = 400;

        armParams.pivotTicksPerDegree = 32.06;
        armParams.elevatorTicksPerInch = 157.86;

        Robot24.ExtendedParameters extendedParams = new Robot24.ExtendedParameters();
        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0.023, 0, 0, 0);

        Robot24.Parameters params = new Robot24.Parameters(clawParams, intakeParams, imuParams, driveParams, armParams, extendedParams);

        return params;
    }

// ---------------------------------------------------------------------------------------------------------------------

//    public static Robot.Parameters getVirtualRobotParams(HardwareMap hardwareMap) {
//
//        Claw.Parameters clawParams = new Claw.Parameters(false, 0.1, 0.01);
//
//
//        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters(true, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
//
//
//        // the defaults in DriveConstants should work here
//        RRMecanumDrive.DriveConstants dc = new RRMecanumDrive.DriveConstants();
//
//        // These are NOT for a goBILDA Strafer! virtual_robot uses these different values.
//        dc.TICKS_PER_REV = 1120;
//        dc.MAX_RPM = 133.9;
//        dc.WHEEL_RADIUS = 2;
//        dc.TRACK_WIDTH = 18;
//        dc.MAX_VEL = dc.MAX_RPM * Math.PI * (dc.WHEEL_RADIUS * 2) / 60.0;
//        dc.kV = 1.0 / dc.MAX_VEL;
//        dc.kA = 0;
//        dc.kStatic = 0;
//
//        RRMecanumDrive.Parameters driveParams = new RRMecanumDrive.Parameters(
//                true,
//                dc,
//                new AccelLimiter(2.0, 0.1)
//        );
//
//        driveParams.useAccelLimiter = true;
//
//        driveParams.leftFrontMotor = hardwareMap.get(DcMotorEx.class, "front_left_motor");
//        driveParams.leftFrontMotor.setDirection(REVERSE);
//
//        driveParams.rightFrontMotor = hardwareMap.get(DcMotorEx.class, "front_right_motor");
//        driveParams.rightFrontMotor.setDirection(FORWARD);
//
//        driveParams.leftBackMotor = hardwareMap.get(DcMotorEx.class, "back_left_motor");
//        driveParams.leftBackMotor.setDirection(REVERSE);
//
//        driveParams.rightBackMotor = hardwareMap.get(DcMotorEx.class, "back_right_motor");
//        driveParams.rightBackMotor.setDirection(FORWARD);
//
//        driveParams.runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
//        driveParams.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
//
//        driveParams.stopwatch = new ElapsedTime();
//
//        driveParams.TRANSLATIONAL_PID = new PIDCoefficients(1, 0, 0);
//        driveParams.HEADING_PID = new PIDCoefficients(1, 0, 0);
//
//
//        PixelArm.Parameters armParams = new PixelArm.Parameters(false);
//
//
//        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters planeLauncherParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters(false, 1, 0);
//
//
//        Robot.ExtendedParameters extendedParams = new Robot.ExtendedParameters();
//        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0.023, 0, 0, 0);
//
//
//        Robot.Parameters params = new Robot.Parameters(clawParams, imuParams, driveParams, armParams, planeLauncherParams, extendedParams);
//        params.tpr = 1120;
//        params.wheelDiameter = 0.1016; // in meters
//        params.maxForwardSpeed = 1.50; // in meters per second
//
//        return params;
//    }

// ---------------------------------------------------------------------------------------------------------------------

//    public static Robot.Parameters getProgrammingBoardParams(HardwareMap hardwareMap) {
//        // Only one motor and one servo can be used at a time. Un-comment the hardwareMap.get() line you need at the
//        // moment, and comment out any others.
//
//        Claw.Parameters clawParams = new Claw.Parameters(true, 0.1, 0.01);
////        clawParams.clawServo = hardwareMap.get(Servo.class, "servo");
//
//        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU.Parameters imuParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU.Parameters(true, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
//
//
//        RRMecanumDrive.DriveConstants dc = new RRMecanumDrive.DriveConstants();
//        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive.Parameters driveParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive.Parameters(
//                true,
//                dc,
//                new AccelLimiter(2.0, 0.1)
//        );
//
////        driveParams.leftFrontMotor = hardwareMap.get(DcMotorEx.class, "motor");
////        driveParams.rightFrontMotor = hardwareMap.get(DcMotorEx.class, "motor");
////        driveParams.leftBackMotor = hardwareMap.get(DcMotorEx.class, "motor");
////        driveParams.rightBackMotor = hardwareMap.get(DcMotorEx.class, "motor");
//
//        driveParams.runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
//        driveParams.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
//
//
//        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm.Parameters armParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm.Parameters(true);
////        armParams.pivotMotor = hardwareMap.get(DcMotorEx.class, "motor");
////        armParams.elevatorMotor = hardwareMap.get(DcMotorEx.class, "motor");
//        armParams.pivotAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
//        armParams.pivotTicksPerDegree = 10;
//        armParams.pivotUpperLimit = 2000;
//        armParams.pivotLowerLimit = 0;
//        armParams.maxPivotRecoveryPower = 0.2;
//        armParams.maxPivotVelocity = 800;
//
//        armParams.elevatorAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
//        armParams.elevatorTicksPerMillimeter = 10;
//        armParams.elevatorUpperLimit = 2500;
//        armParams.elevatorLowerLimit = 0;
//        armParams.maxElevatorRecoveryPower = 0.2;
//        armParams.maxElevatorVelocity = 800;
//
//
//        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters planeLauncherParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters(true, 1, 0);
////        planeLauncherParams.planeServo = hardwareMap.get(Servo.class,"servo");
//
//        Robot.ExtendedParameters extendedParams = new Robot.ExtendedParameters();
//        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0, 0, 0, 0);
//
//        Robot.Parameters params = new Robot.Parameters(clawParams, imuParams, driveParams, armParams, planeLauncherParams, extendedParams);
//        params.tpr = 537.7; // ticks per rotation
//        params.wheelDiameter = 0.096; // in meters
//        params.maxForwardSpeed = 1.50; // in meters per second
//
//        return params;
//    }



}
