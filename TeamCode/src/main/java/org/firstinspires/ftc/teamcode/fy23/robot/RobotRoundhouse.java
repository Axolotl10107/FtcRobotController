package org.firstinspires.ftc.teamcode.fy23.robot;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.*;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.digitaldevice.DigitalDeviceBlank;
import org.firstinspires.ftc.teamcode.fy23.units.PIDConsts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class RobotRoundhouse {

    public static Robot.Parameters getRobotAParams(HardwareMap hardwareMap) {

        Claw.Parameters clawParams = new Claw.Parameters(true, 0.1, 0.01);
        clawParams.clawServo = hardwareMap.get(Servo.class, "clawServo");

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters(true, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        RRMecanumDrive.DriveConstants dc = new RRMecanumDrive.DriveConstants();

        dc.TICKS_PER_REV = 537.7;
        dc.MAX_RPM = 312;
        dc.RUN_USING_ENCODER = true;
        dc.MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
                dc.getMotorVelocityF(dc.MAX_RPM / 60 * dc.TICKS_PER_REV));

        dc.WHEEL_RADIUS = 1.88976;
        dc.GEAR_RATIO = 1;
        dc.TRACK_WIDTH = 16.25;

        dc.kV = 1.0 / dc.rpmToVelocity(dc.MAX_RPM);
        dc.kA = 0;
        dc.kStatic = 0;

        // TODO: Tune these!
        dc.MAX_VEL = 50;
        dc.MAX_ACCEL = 50;
        dc.MAX_ANG_VEL = Math.toRadians(60);
        dc.MAX_ANG_ACCEL = Math.toRadians(60);

        RRMecanumDrive.Parameters driveParams = new RRMecanumDrive.Parameters(true,
                dc,
                new AccelLimiter(2.0, 0.1));

        driveParams.TRANSLATIONAL_PID = new PIDCoefficients(1, 0, 0);
        driveParams.HEADING_PID = new PIDCoefficients(1, 0, 0);
        driveParams.LATERAL_MULTIPLIER = 1;
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

        PixelArm.Parameters armParams = new PixelArm.Parameters(true);

        armParams.pivotMotor = hardwareMap.get(DcMotorEx.class, "armPivot");
        armParams.pivotMotor.setDirection(REVERSE);
        armParams.pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armParams.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armParams.pivotAccelLimiter = new AccelLimiter(12000, 8000); // TODO: not tuned!!
        armParams.pivotTicksPerDegree = 10; // TODO: not measured!!

        armParams.pivotUpperLimit = 3800; // correct value is 3800
        armParams.pivotLowerLimit = 0;
        armParams.pivotUpperLimitSwitch = new DigitalDeviceBlank(); // not installed
        armParams.pivotLowerLimitSwitch = new DigitalDeviceBlank(); // not installed

        armParams.maxPivotRecoveryPower = 0.2;
        armParams.maxPivotVelocity = 1200; // correct value is 2400


        armParams.elevatorMotor = hardwareMap.get(DcMotorEx.class, "armExtend");
        armParams.elevatorMotor.setDirection(REVERSE);
        armParams.elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armParams.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armParams.elevatorAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
        armParams.elevatorTicksPerMillimeter = 10; // TODO: not measured!!

        armParams.elevatorUpperLimit = 2500;
        armParams.elevatorLowerLimit = 0;
        armParams.elevatorUpperLimitSwitch = new DigitalDeviceBlank(); // not installed
        armParams.elevatorLowerLimitSwitch = new DigitalDeviceBlank(); // not installed

        armParams.maxElevatorRecoveryPower = 0.2;
        armParams.maxElevatorVelocity = 800; // TODO: not measured!!

        armParams.stopwatch = new ElapsedTime();

        PlaneLauncher.Parameters planeLauncherParams = new PlaneLauncher.Parameters(true, 1, 0);
        planeLauncherParams.planeServo = hardwareMap.get(Servo.class,"planeservo");

        Robot.ExtendedParameters extendedParams = new Robot.ExtendedParameters();
        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0.023, 0, 0,0 );

        Robot.Parameters params = new Robot.Parameters(clawParams, imuParams, driveParams, armParams, planeLauncherParams, extendedParams);
        params.tpr = 537.7; // ticks per rotation
        params.wheelDiameter = 0.096; // in meters
        params.maxForwardSpeed = 1.50; // in meters per second

        return params;
    }

// ---------------------------------------------------------------------------------------------------------------------

    public static Robot.Parameters getRobotBParams(HardwareMap hardwareMap) {

        Claw.Parameters clawParams = new Claw.Parameters(false, 0.1, 0.01);

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters(true, RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);

        RRMecanumDrive.DriveConstants dc = new RRMecanumDrive.DriveConstants();
        dc.LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        dc.USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RRMecanumDrive.Parameters driveParams = new RRMecanumDrive.Parameters(true,
                dc, // most of the defaults in DriveConstants should work here
                new AccelLimiter(2.0, 0.1));
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

        // the rest of the defaults in RRMecanumDrive.Parameters should work here

        PixelArm.Parameters armParams = new PixelArm.Parameters(false);

        PlaneLauncher.Parameters planeLauncherParams = new PlaneLauncher.Parameters(false, 1, 0);

        Robot.ExtendedParameters extendedParams = new Robot.ExtendedParameters();
        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0.023, 0, 0, 0);

        Robot.Parameters params = new Robot.Parameters(clawParams, imuParams, driveParams, armParams, planeLauncherParams, extendedParams);
        params.tpr = 537.7;
        params.wheelDiameter = 0.096; // in meters
        params.maxForwardSpeed = 1.50; // in meters per second

        return params;
    }

// ---------------------------------------------------------------------------------------------------------------------

    public static Robot.Parameters getVirtualRobotParams(HardwareMap hardwareMap) {

        Claw.Parameters clawParams = new Claw.Parameters(false, 0.1, 0.01);


        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters(true, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);


        // the defaults in DriveConstants should work here
        RRMecanumDrive.DriveConstants dc = new RRMecanumDrive.DriveConstants();

        // These are NOT for a goBILDA Strafer! virtual_robot uses these different values.
        dc.TICKS_PER_REV = 1120;
        dc.MAX_RPM = 133.9;
        dc.WHEEL_RADIUS = 2;
        dc.TRACK_WIDTH = 18;
        dc.MAX_VEL = dc.MAX_RPM * Math.PI * (dc.WHEEL_RADIUS * 2) / 60.0;
        dc.kV = 1.0 / dc.MAX_VEL;
        dc.kA = 0;
        dc.kStatic = 0;

        RRMecanumDrive.Parameters driveParams = new RRMecanumDrive.Parameters(
                true,
                dc,
                new AccelLimiter(2.0, 0.1)
        );

        driveParams.useAccelLimiter = true;

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

        driveParams.stopwatch = new ElapsedTime();

        driveParams.TRANSLATIONAL_PID = new PIDCoefficients(1, 0, 0);
        driveParams.HEADING_PID = new PIDCoefficients(1, 0, 0);


        PixelArm.Parameters armParams = new PixelArm.Parameters(false);


        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters planeLauncherParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters(false, 1, 0);


        Robot.ExtendedParameters extendedParams = new Robot.ExtendedParameters();
        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0.023, 0, 0, 0);


        Robot.Parameters params = new Robot.Parameters(clawParams, imuParams, driveParams, armParams, planeLauncherParams, extendedParams);
        params.tpr = 1120;
        params.wheelDiameter = 0.1016; // in meters
        params.maxForwardSpeed = 1.50; // in meters per second

        return params;
    }

// ---------------------------------------------------------------------------------------------------------------------

    public static Robot.Parameters getProgrammingBoardParams(HardwareMap hardwareMap) {
        // Only one motor and one servo can be used at a time. Un-comment the hardwareMap.get() line you need at the
        // moment, and comment out any others.

        Claw.Parameters clawParams = new Claw.Parameters(true, 0.1, 0.01);
//        clawParams.clawServo = hardwareMap.get(Servo.class, "servo");

        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU.Parameters imuParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU.Parameters(true, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);


        RRMecanumDrive.DriveConstants dc = new RRMecanumDrive.DriveConstants();
        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive.Parameters driveParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive.Parameters(
                true,
                dc,
                new AccelLimiter(2.0, 0.1)
        );

//        driveParams.leftFrontMotor = hardwareMap.get(DcMotorEx.class, "motor");
//        driveParams.rightFrontMotor = hardwareMap.get(DcMotorEx.class, "motor");
//        driveParams.leftBackMotor = hardwareMap.get(DcMotorEx.class, "motor");
//        driveParams.rightBackMotor = hardwareMap.get(DcMotorEx.class, "motor");

        driveParams.runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        driveParams.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;


        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm.Parameters armParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm.Parameters(true);
//        armParams.pivotMotor = hardwareMap.get(DcMotorEx.class, "motor");
//        armParams.elevatorMotor = hardwareMap.get(DcMotorEx.class, "motor");
        armParams.pivotAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
        armParams.pivotTicksPerDegree = 10;
        armParams.pivotUpperLimit = 2000;
        armParams.pivotLowerLimit = 0;
        armParams.maxPivotRecoveryPower = 0.2;
        armParams.maxPivotVelocity = 800;

        armParams.elevatorAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
        armParams.elevatorTicksPerMillimeter = 10;
        armParams.elevatorUpperLimit = 2500;
        armParams.elevatorLowerLimit = 0;
        armParams.maxElevatorRecoveryPower = 0.2;
        armParams.maxElevatorVelocity = 800;


        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters planeLauncherParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters(true, 1, 0);
//        planeLauncherParams.planeServo = hardwareMap.get(Servo.class,"servo");

        Robot.ExtendedParameters extendedParams = new Robot.ExtendedParameters();
        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0, 0, 0, 0);

        Robot.Parameters params = new Robot.Parameters(clawParams, imuParams, driveParams, armParams, planeLauncherParams, extendedParams);
        params.tpr = 537.7; // ticks per rotation
        params.wheelDiameter = 0.096; // in meters
        params.maxForwardSpeed = 1.50; // in meters per second

        return params;
    }

}
