package org.firstinspires.ftc.teamcode.fy23.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.FriendlyIMUImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PixelArmImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PlaneLauncherImpl;
import org.firstinspires.ftc.teamcode.fy23.units.PIDconsts;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class RobotRoundhouse {

    public static Robot.Parameters getRobotAParams() {
        Claw.Parameters clawParams = new Claw.Parameters();
        clawParams.present = true;
        clawParams.clawServoName = "clawServo";
        clawParams.openPosition = 0.1;
        clawParams.closePosition = 0.01;

        FriendlyIMUImpl.Parameters imuParams = new FriendlyIMUImpl.Parameters();
        imuParams.present = true;

        MecanumDriveImpl.Parameters driveParams = new MecanumDriveImpl.Parameters();
        driveParams.present = true;

        driveParams.maxMotorAccel = 2; // meters per second
        driveParams.maxDeltaVEachLoop = 0.1;

        driveParams.leftFrontName = "leftFront";
        driveParams.leftFrontDirection = REVERSE;

        driveParams.rightFrontName = "rightFront";
        driveParams.rightFrontDirection = FORWARD;

        driveParams.leftBackName = "leftBack";
        driveParams.leftBackDirection = REVERSE;

        driveParams.rightBackName = "rightBack";
        driveParams.rightBackDirection = FORWARD;

        driveParams.runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        driveParams.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

        PixelArmImpl.Parameters armParams = new PixelArmImpl.Parameters();
        armParams.present = true;
        armParams.pivotMotorName = "armPivot";
        armParams.elevatorMotorName = "armExtend";
        armParams.maxPivotAccel = 1.0; // meters per second
        armParams.maxPivotDeltaVEachLoop = 0.1;
        armParams.pivotUpperLimit = 2000;
        armParams.pivotLowerLimit = 0;
        armParams.maxElevatorAccel = 1.0;
        armParams.maxElevatorDeltaVEachLoop = 0.1;
        armParams.elevatorUpperLimit = 2500;
        armParams.elevatorLowerLimit = 0;

        PlaneLauncherImpl.Parameters planeLauncherParams = new PlaneLauncherImpl.Parameters();
        planeLauncherParams.present = true;
        planeLauncherParams.planeServoName = "planeServo";

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

    public static Robot.Parameters getRobotBParams() {
        Claw.Parameters clawParams = new Claw.Parameters();
        clawParams.present = false;

        FriendlyIMUImpl.Parameters imuParams = new FriendlyIMUImpl.Parameters();
        imuParams.present = true;

        MecanumDriveImpl.Parameters driveParams = new MecanumDriveImpl.Parameters();
        driveParams.present = true;

        driveParams.maxMotorAccel = 2.0;
        driveParams.maxDeltaVEachLoop = 0.1;

        driveParams.leftFrontName = "leftFront";
        driveParams.leftFrontDirection = REVERSE;

        driveParams.rightFrontName = "rightFront";
        driveParams.rightFrontDirection = FORWARD;

        driveParams.leftBackName = "leftBack";
        driveParams.leftBackDirection = REVERSE;

        driveParams.rightBackName = "rightBack";
        driveParams.rightBackDirection = FORWARD;

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

    public static Robot.Parameters getVirtualRobotParams() {
        Claw.Parameters clawParams = new Claw.Parameters();
        clawParams.present = false;

        FriendlyIMUImpl.Parameters imuParams = new FriendlyIMUImpl.Parameters();
        imuParams.present = true;

        MecanumDriveImpl.Parameters driveParams = new MecanumDriveImpl.Parameters();
        driveParams.present = true;

        driveParams.maxMotorAccel = 1.0;
        driveParams.maxDeltaVEachLoop = 0.1;

        driveParams.leftFrontName = "front_left_motor";
        driveParams.leftFrontDirection = REVERSE;

        driveParams.rightFrontName = "front_right_motor";
        driveParams.rightFrontDirection = FORWARD;

        driveParams.leftBackName = "back_left_motor";
        driveParams.leftBackDirection = REVERSE;

        driveParams.rightBackName = "back_right_motor";
        driveParams.rightBackDirection = FORWARD;

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

}
