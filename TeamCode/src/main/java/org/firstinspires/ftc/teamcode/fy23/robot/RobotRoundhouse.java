package org.firstinspires.ftc.teamcode.fy23.robot;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PixelArm;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PlaneLauncher;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.Claw;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.FriendlyIMU;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.MecanumDrive;
import org.firstinspires.ftc.teamcode.fy23.robot.units.PIDconsts;

public class RobotRoundhouse {

    public static Robot.Parameters getRobotAParams() {
        Claw.Parameters clawParams = new Claw.Parameters();
        clawParams.present = true;

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters();
        imuParams.present = true;

        MecanumDrive.Parameters driveParams = new MecanumDrive.Parameters();
        driveParams.leftFrontName = "leftFront";
        driveParams.leftFrontDirection = REVERSE;

        driveParams.rightFrontName = "rightFront";
        driveParams.rightFrontDirection = FORWARD;

        driveParams.leftBackName = "leftBack";
        driveParams.leftBackDirection = REVERSE;

        driveParams.rightBackName = "rightBack";
        driveParams.rightBackDirection = FORWARD;

        PixelArm.Parameters armParams = new PixelArm.Parameters();
        armParams.present = true;

        PlaneLauncher.Parameters planeLauncherParams = new PlaneLauncher.Parameters();
        planeLauncherParams.present = true;

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

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters();
        imuParams.present = true;

        MecanumDrive.Parameters driveParams = new MecanumDrive.Parameters();
        driveParams.leftFrontName = "leftFront";
        driveParams.leftFrontDirection = REVERSE;

        driveParams.rightFrontName = "rightFront";
        driveParams.rightFrontDirection = FORWARD;

        driveParams.leftBackName = "leftBack";
        driveParams.leftBackDirection = REVERSE;

        driveParams.rightBackName = "rightBack";
        driveParams.rightBackDirection = FORWARD;

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

    public static Robot.Parameters getVirtualRobotParams() {
        Claw.Parameters clawParams = new Claw.Parameters();
        clawParams.present = false;

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters();
        imuParams.present = true;

        MecanumDrive.Parameters driveParams = new MecanumDrive.Parameters();
        driveParams.leftFrontName = "front_left_motor";
        driveParams.leftFrontDirection = REVERSE;

        driveParams.rightFrontName = "front_right_motor";
        driveParams.rightFrontDirection = FORWARD;

        driveParams.leftBackName = "back_left_motor";
        driveParams.leftBackDirection = REVERSE;

        driveParams.rightBackName = "back_right_motor";
        driveParams.rightBackDirection = FORWARD;

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

}
