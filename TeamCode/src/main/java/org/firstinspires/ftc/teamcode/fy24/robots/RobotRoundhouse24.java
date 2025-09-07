package org.firstinspires.ftc.teamcode.fy24.robots;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.network.ControlHubDeviceNameManager;
import org.firstinspires.ftc.teamcode.framework.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.framework.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.fy24.subsystems.doublearm.DoubleArm;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMU;
import org.firstinspires.ftc.teamcode.fy23.subsystems.planelauncher.PlaneLauncher;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.framework.units.PIDConsts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * RobotRoundhouse stores {@link Robot24.Parameters} for every Robot that currently exists.
 * If you need to change a Robot configuration option, change it here.
 * Everything that could potentially differ between any two real-life robots has been split out to
 * a parameter of either a subsystem or the entire Robot.
 * There are many Parameters, and they are all centralized here. Please get them from here instead
 * of making custom ones inside of OpModes; otherwise we'll have multiple mystery copies of
 * something that has over 100 config. options and can cause physical damage if it is incorrect.
 * <p>
 * The name of a robot in the Roundhouse matches the name of its control hub.
 * So, RobotA = 10107-A-RC and RobotB = 10107-B-RC.
 * You can get the parameters of a specific robot, or you can have the Roundhouse automatically
 * determine which robot this is and give you the correct Parameters.
 * <p>
 * NOTE: If you're using virtual_robot, you must (currently) use getVirtualRobotParams().
 * getParamsAuto() only works on real hardware.
 * */
public class RobotRoundhouse24 {

    /** Returns the correct Parameters for the specified robot.
     * NOTE: Doesn't work on virtual_robot.
     * {@param serialNumber} By "serialNumber", the SDK actually means the name you set on the
     * Control Hub. So RobotA's "serialNumber" is "10107-A-RC". Pass in the name of the robot
     * controller you want parameters for.
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
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

    /** Automatically determines which robot you're running on and returns the correct Parameters.
     * NOTE: Doesn't work on virtual_robot.
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public static Robot24.Parameters getParamsAuto(HardwareMap hardwareMap) {
        // ControlHubDeviceNameManager doesn't exist in virtual_robot, so if you're using that, comment this out in your
        // virtual_robot project and have this return the virtualRobot or programmingBoard parameters.
        return getParamsAuto(ControlHubDeviceNameManager.getControlHubDeviceNameManager().getDeviceName(), hardwareMap);
    }

    /** Returns the Parameters for RobotA.
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public static Robot24.Parameters getRobotAParams(HardwareMap hardwareMap) {

        Claw.Parameters clawParams = new Claw.Parameters(false);
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

    /** Returns the Parameters for RobotB.
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public static Robot24.Parameters getRobotBParams(HardwareMap hardwareMap) {

        Claw.Parameters clawParams = new Claw.Parameters(true);
        clawParams.openPosition = 0.1;
        clawParams.closePosition = 0.01;
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

        // Due to a bug somewhere, these values should be 10 times higher than
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

    /** Returns the Parameters for virtual_robot. If you haven't created a robot there that matches
     * your real hardware, you'll probably be enabling/disabling subsystems regularly to test
     * different things on one of the provided bots.
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public static Robot24.Parameters getVirtualRobotParams(HardwareMap hardwareMap) {

        Claw.Parameters clawParams = new Claw.Parameters(false);


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


        DoubleArm.Parameters armParams = new DoubleArm.Parameters(false);

        RotaryIntake.Parameters intakeParams = new RotaryIntake.Parameters(false);


        PlaneLauncher.Parameters planeLauncherParams = new PlaneLauncher.Parameters(false, 1, 0);


        Robot24.ExtendedParameters extendedParams = new Robot24.ExtendedParameters();
        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0.023, 0, 0, 0);


        Robot24.Parameters params = new Robot24.Parameters(clawParams, intakeParams, imuParams, driveParams, armParams, extendedParams);

        // These were removed some time ago, but if you need these values for something else,
        // here's what they are for virtual_robot's Mecanum Bot.
//        params.tpr = 1120;
//        params.wheelDiameter = 0.1016; // in meters
//        params.maxForwardSpeed = 1.50; // in meters per second

        return params;
    }

// ---------------------------------------------------------------------------------------------------------------------

    /** Returns the Parameters for the Programming Board from the book "Learn Java for FTC".
     * virtual_robot provides the Programming Board too. This has usually been used to hook one
     * output from one subsystem to one actuator, so these Parameters get messed with all the time.
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public static Robot24.Parameters getProgrammingBoardParams(HardwareMap hardwareMap) {
        // Only one motor and one servo can be used at a time. Un-comment the hardwareMap.get() line you need at the
        // moment, and comment out any others.
        // For all subsystems, if the Parameter for an actuator is left blank, it defaults to a
        // blank actuator. For example, commenting out the clawServo parameter causes the Claw
        // subsystem to replace it with a BlankServo. This won't cause issues; the functionality of
        // that actuator will just be disabled.

        Claw.Parameters clawParams = new Claw.Parameters(true);
        clawParams.openPosition = 0.1;
        clawParams.closePosition = 0.01;
//        clawParams.clawServo = hardwareMap.get(Servo.class, "servo");

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters(true, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);


        RRMecanumDrive.DriveConstants dc = new RRMecanumDrive.DriveConstants();
        RRMecanumDrive.Parameters driveParams = new RRMecanumDrive.Parameters(
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


        DoubleArm.Parameters armParams = new DoubleArm.Parameters(true);
//        armParams.pivotMotor = hardwareMap.get(DcMotorEx.class, "motor");
//        armParams.elevatorMotor = hardwareMap.get(DcMotorEx.class, "motor");
        armParams.pivotAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
        armParams.pivotTicksPerDegree = 10;
        armParams.pivotUpperLimit = 2000;
        armParams.pivotLowerLimit = 0;
        armParams.maxPivotRecoveryPower = 0.2;
        armParams.maxPivotVelocity = 800;

        armParams.elevatorAccelLimiter = new AccelLimiter(1.0, 0.1); // TODO: not tuned!!
//        armParams.elevatorTicksPerMillimeter = 10;
        armParams.elevatorTicksPerInch = 254; // 1 in. = 25.4 mm
        armParams.elevatorUpperLimit = 2500;
        armParams.elevatorLowerLimit = 0;
        armParams.maxElevatorRecoveryPower = 0.2;
        armParams.maxElevatorVelocity = 800;


//        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters planeLauncherParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters(true, 1, 0);
//        planeLauncherParams.planeServo = hardwareMap.get(Servo.class,"servo");

        RotaryIntake.Parameters intakeParams = new RotaryIntake.Parameters(false);

        Robot24.ExtendedParameters extendedParams = new Robot24.ExtendedParameters();
        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0, 0, 0, 0);

        Robot24.Parameters params = new Robot24.Parameters(clawParams, intakeParams, imuParams, driveParams, armParams, extendedParams);
//        params.tpr = 537.7; // ticks per rotation
//        params.wheelDiameter = 0.096; // in meters
//        params.maxForwardSpeed = 1.50; // in meters per second

        return params;
    }



}
