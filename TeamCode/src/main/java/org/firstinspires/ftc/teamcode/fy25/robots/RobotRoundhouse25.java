package org.firstinspires.ftc.teamcode.fy25.robots;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.network.ControlHubDeviceNameManager;
import org.firstinspires.ftc.teamcode.framework.adapters.DualCRServo;
import org.firstinspires.ftc.teamcode.framework.adapters.DualDcMotorEx;
import org.firstinspires.ftc.teamcode.framework.adapters.DualMotor;
import org.firstinspires.ftc.teamcode.framework.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.framework.units.PIDConsts;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motorintake.MotorIntake;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate.LauncherGate;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel.LauncherWheel;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * RobotRoundhouse stores {@link Robot25.Parameters} for every Robot that currently exists.
 * If you need to change a Robot configuration option, change it here.
 * Everything that could potentially differ between any two real-life robots has been split out to
 * a parameter of either a subsystem or the entire Robot.
 * There are many Parameters, and they are all centralized here. Please get them from here instead
 * of making custom ones inside of OpModes; otherwise we'll have multiple mystery copies of
 * something that has over 100 configuration options and can cause physical damage if it is incorrect.
 * <p>
 * The name of a robot in the Roundhouse matches the name of its control hub.
 * So, RobotA = '10107-A-RC' and RobotB = '10107-B-RC'.
 * You can get the parameters of a specific robot, or you can have the Roundhouse automatically
 * determine which robot this is and give you the correct Parameters.
 * Or, new this year, use getRobotAuto() to get a Robot object.
 * <p>
 * NOTE: If you're using virtual_robot, you must (currently) use getVirtualRobotParams().
 * getParamsAuto() only works on real hardware.
 * */
public class RobotRoundhouse25 {

    public static class OldRobotException extends Exception {
        public OldRobotException(String message) {
            super(message);
        }
        public OldRobotException(String message, Throwable cause) {
            super(message, cause);
        }
    }

    /** Returns the correct Parameters for the specified robot.
     * NOTE: Doesn't work on virtual_robot.
     * {@param serialNumber} By "serialNumber", the SDK actually means the name you set on the
     * Control Hub. So RobotA's "serialNumber" is "10107-A-RC". Pass in the name of the robot
     * controller you want parameters for.
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public static Robot25.Parameters getParamsAuto(String serialNumber, HardwareMap hardwareMap) throws OldRobotException {
        switch (serialNumber) {
            case "10107-A-RC":
                return getRobotAParams(hardwareMap);
            case "10107-B-RC":
                return getRobotBParams(hardwareMap);
            default:
                return getRobotAParams(hardwareMap);
        }
    }

    /** Detects which robot you're running on and returns the correct Parameters.
     * NOTE: Doesn't work on virtual_robot.
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public static Robot25.Parameters getParamsAuto(HardwareMap hardwareMap) throws OldRobotException {
        // ControlHubDeviceNameManager doesn't exist in virtual_robot, so if you're using that, comment this out in your
        // virtual_robot project and have this return the virtualRobot or programmingBoard parameters.
        return getParamsAuto(ControlHubDeviceNameManager.getControlHubDeviceNameManager().getDeviceName(), hardwareMap);
    }

    /** Detects which physical robot you're running on and returns the correct Robot Object.
     * Note: Doesn't work on virtual_robot.
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public static Robot25 getRobotAuto(HardwareMap hardwareMap) throws OldRobotException {
        return new Robot25(RobotRoundhouse25.getParamsAuto(hardwareMap), hardwareMap);
    }

    /** Returns the Parameters for RobotA.
     * RobotA is currently the fy25 small bot.
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public static Robot25.Parameters getRobotAParams(HardwareMap hardwareMap) {

        RRMecanumDrive.DriveConstants dc = new RRMecanumDrive.DriveConstants();

        dc.TICKS_PER_REV = 537.7;
        dc.MAX_RPM = 117; // TODO: Verify this, usually 312 so this is strange
        dc.RUN_USING_ENCODER = true;
        dc.MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
                dc.getMotorVelocityF(dc.MAX_RPM / 60 * dc.TICKS_PER_REV));

        dc.WHEEL_RADIUS = 1.88976;
        dc.GEAR_RATIO = 1;
//        dc.TRACK_WIDTH = 15.9;
//        dc.TRACK_WIDTH = 14; // change for small chassis
        dc.TRACK_WIDTH = 13.75; // attempted change relocated from RRMecanumDrive 11-08-25

        dc.kV = .017;
        dc.kA = .002;
        dc.kStatic = 0.0001;

        dc.MAX_ANG_VEL = 3;
        dc.MAX_ANG_ACCEL = 3;

        dc.MAX_VEL = 30;
        dc.MAX_ACCEL = 30;

        RRMecanumDrive.Parameters driveParams = new RRMecanumDrive.Parameters(
                true,
                dc,
                new AccelLimiter(2.0, 0.1)
        );

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


        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters(
                true,
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );


        LauncherWheel.Parameters launchWheelParams = new LauncherWheel.Parameters(true);
        DcMotorEx launchWheelMotor = new DualDcMotorEx(
                hardwareMap.get(DcMotorEx.class, "launchWheelMotorFront"),
                hardwareMap.get(DcMotorEx.class, "launchWheelMotorBack")
        );
        launchWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchWheelParams.motor = launchWheelMotor;
        launchWheelParams.isDynamic = false;
        // TODO: Tune default launch wheel velocity
        launchWheelParams.velocityRPM = 6000;
        launchWheelParams.velocityTolerance = 20;
        launchWheelParams.motorTPR = 28;
        launchWheelParams.denyVel = -500;

        LauncherGate.Parameters launchGateParams = new LauncherGate.Parameters(true);
        launchGateParams.device = hardwareMap.get(DcMotorSimple.class, "launchGateMotor");
        launchGateParams.power = 1;

        MotorIntake.Parameters motorIntakeParams = new MotorIntake.Parameters(true);
        motorIntakeParams.motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        motorIntakeParams.motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorIntakeParams.IntakeTPS = 166865;

        Robot25.ExtendedParameters extendedParams = new Robot25.ExtendedParameters();
        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0.023, 0, 0,0 );

        Robot25.Parameters params = new Robot25.Parameters(
                extendedParams,
                driveParams,
                imuParams,

                launchWheelParams,
                launchGateParams,
                motorIntakeParams
        );

        return params;
    }

// ---------------------------------------------------------------------------------------------------------------------

    /** Returns the Parameters for RobotB.
     * RobotB is the Starter Bot from Scrap (so make these match that testbed, <i>not</i> the competition robot!).
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public static Robot25.Parameters getRobotBParams(HardwareMap hardwareMap) {

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters(
                true,
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        RRMecanumDrive.DriveConstants dc = new RRMecanumDrive.DriveConstants();
        dc.LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        dc.USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RRMecanumDrive.Parameters driveParams = new RRMecanumDrive.Parameters(
                true,
                dc, // most of the defaults in DriveConstants should work here
                new AccelLimiter(4.0, 0.3)
        );
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


        LauncherWheel.Parameters launchWheelParams = new LauncherWheel.Parameters(true);
        DcMotorEx launchWheelMotor = hardwareMap.get(DcMotorEx.class, "launchWheelMotor");
        launchWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchWheelParams.motor = launchWheelMotor;
        launchWheelParams.velocityRPM = 6000;
        launchWheelParams.velocityTolerance = 20;
        launchWheelParams.motorTPR = 28;
        launchWheelParams.isDynamic = false;
        launchWheelParams.denyVel = -500;

        LauncherGate.Parameters launchGateParams = new LauncherGate.Parameters(true);
        CRServo servoLeft = hardwareMap.get(CRServo.class, "launchGateServoLeft");
        CRServo servoRight = hardwareMap.get(CRServo.class, "launchGateServoRight");
        servoLeft.setDirection(REVERSE);
        launchGateParams.device = new DualCRServo(servoRight, servoLeft);


        MotorIntake.Parameters motorIntakeParams = new MotorIntake.Parameters(true);
        motorIntakeParams.motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        motorIntakeParams.motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorIntakeParams.IntakeTPS = 537;


        Robot25.ExtendedParameters extendedParams = new Robot25.ExtendedParameters();
        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0.023, 0, 0, 0);


        Robot25.Parameters params = new Robot25.Parameters(
                extendedParams,
                driveParams,
                imuParams,

                launchWheelParams,
                launchGateParams,
                motorIntakeParams
        );

        return params;
    }

// ---------------------------------------------------------------------------------------------------------------------

    /** Returns the Parameters for virtual_robot. If you haven't created a robot there that matches
     * your real hardware, you'll probably be enabling/disabling subsystems regularly to test
     * different things on one of the provided bots.
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public static Robot25.Parameters getVirtualRobotParams(HardwareMap hardwareMap) {


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


        Robot25.ExtendedParameters extendedParams = new Robot25.ExtendedParameters();
        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0.023, 0, 0, 0);


        LauncherWheel.Parameters launchWheelParams = new LauncherWheel.Parameters(false);


        LauncherGate.Parameters launchGateParams = new LauncherGate.Parameters(false);

        MotorIntake.Parameters motorIntakeParams = new MotorIntake.Parameters(false);


        Robot25.Parameters params = new Robot25.Parameters(
                extendedParams,
                driveParams,
                imuParams,

                launchWheelParams,
                launchGateParams,
                motorIntakeParams
        );

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
    public static Robot25.Parameters getProgrammingBoardParams(HardwareMap hardwareMap) {
        // Only one motor and one servo can be used at a time. Un-comment the hardwareMap.get() line you need at the
        // moment, and comment out any others.
        // For all subsystems, if the Parameter for an actuator is left blank, it defaults to a
        // blank actuator. For example, commenting out the clawServo parameter causes the Claw
        // subsystem to replace it with a BlankServo. This won't cause issues; the functionality of
        // that actuator will just be disabled.

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


        LauncherWheel.Parameters launchWheelParams = new LauncherWheel.Parameters(true);
        launchWheelParams.velocityRPM = 6000;
        launchWheelParams.motor = hardwareMap.get(DcMotorEx.class, "motor");
//         LauncherWheel.Parameters launchWheelParams = new LauncherWheel.Parameters(false);


        LauncherGate.Parameters launchGateParams = new LauncherGate.Parameters(true);
//        launchGateParams.deviceClass = CRServo.class;
        launchGateParams.device = hardwareMap.get(CRServo.class, "servo");
//        LauncherGate.Parameters launchGateParams = new LauncherGate.Parameters(false);

//        MotorIntake.Parameters motorIntakeParams = new MotorIntake.Parameters(true);
//        motorIntakeParams.motor = hardwareMap.get(DcMotorEx.class, "motor");
//        motorIntakeParams.IntakeTPS = 537;
        MotorIntake.Parameters motorIntakeParams = new MotorIntake.Parameters(false);


        Robot25.ExtendedParameters extendedParams = new Robot25.ExtendedParameters();
        extendedParams.hdgCorrectionPIDConsts = new PIDConsts(0, 0, 0, 0);

        Robot25.Parameters params = new Robot25.Parameters(
                extendedParams,
                driveParams,
                imuParams,

                launchWheelParams,
                launchGateParams,
                motorIntakeParams
        );
//        params.tpr = 537.7; // ticks per rotation
//        params.wheelDiameter = 0.096; // in meters
//        params.maxForwardSpeed = 1.50; // in meters per second

        return params;
    }



}
