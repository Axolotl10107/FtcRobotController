package org.firstinspires.ftc.teamcode.fy23.robot;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.*;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.BlankMotor;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.digitaldevice.DigitalDeviceBlank;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive;
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

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters();
        imuParams.present = true;

        RRMecanumDrive.Parameters driveParams = new RRMecanumDrive.Parameters();
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

        PixelArm.Parameters armParams = new PixelArm.Parameters();
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

        PlaneLauncher.Parameters planeLauncherParams = new PlaneLauncher.Parameters();
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

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters();
        imuParams.present = true;

        RRMecanumDrive.Parameters driveParams = new RRMecanumDrive.Parameters();
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

        PixelArm.Parameters armParams = new PixelArm.Parameters();
        armParams.present = false;

        PlaneLauncher.Parameters planeLauncherParams = new PlaneLauncher.Parameters();
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

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters();
        imuParams.present = true;

        RRMecanumDrive.Parameters driveParams = new RRMecanumDrive.Parameters();
        RRMecanumDrive.DriveConstants dc = new RRMecanumDrive.DriveConstants();

        driveParams.TRANSLATIONAL_PID = new PIDCoefficients(1, 0, 0);
        driveParams.HEADING_PID = new PIDCoefficients(1, 0, 0);
        dc.TICKS_PER_REV = 1120;
        dc.MAX_RPM = 133.9;
        dc.WHEEL_RADIUS = 2;
        dc.TRACK_WIDTH = 18;
        dc.MAX_VEL = dc.MAX_RPM * Math.PI * (dc.WHEEL_RADIUS * 2) / 60.0;
        dc.kV = 1.0 / dc.MAX_VEL;
        dc.kA = 0;
        dc.kStatic = 0;

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

        PixelArm.Parameters armParams = new PixelArm.Parameters();
        armParams.present = false;

        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters planeLauncherParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters();
        planeLauncherParams.present = false;

        Robot.Parameters params = new Robot.Parameters();
        params.tpr = 1120;
        params.wheelDiameter = 0.1016; // in meters
        params.maxForwardSpeed = 1.50; // in meters per second
        params.hdgCorrectionPIDconsts = new PIDconsts(0, 0, 0);

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

        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU.Parameters imuParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU.Parameters();
        imuParams.present = false;

        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive.Parameters driveParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive.Parameters();
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

        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm.Parameters armParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm.Parameters();
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

        org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters planeLauncherParams = new org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher.Parameters();
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
