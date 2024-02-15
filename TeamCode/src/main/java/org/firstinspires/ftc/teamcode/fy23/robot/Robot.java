package org.firstinspires.ftc.teamcode.fy23.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.fy23.robot.units.PIDconsts;

public class Robot {

    public static class Parameters {
        double TPR;
        double wheelDiameter;
        double maxForwardSpeed;
        double driveToStrafeDistCV; // conversion factor from driving distance to equivalent
        // strafing distance, in encoder ticks
        PIDconsts hdgCorrectionPIDconsts;

        Claw.Parameters clawParameters;
        FriendlyIMU.Parameters imuParameters;
        MecanumDrive.Parameters driveParameters;
        PixelArm.Parameters pixelArmParameters;
        PlaneLauncher.Parameters planeLauncherParameters;
    }

    public final double TPR;
    public final double wheelDiameter;
    public final double maxForwardSpeed;
    public final PIDconsts hdgCorrectionPIDconsts;

    public Claw claw;
    public FriendlyIMU imu;
    public MecanumDrive drive;
    public PixelArm arm;
    public PlaneLauncher planeLauncher;

    public Robot(Parameters parameters, HardwareMap hardwareMap) {
        TPR = parameters.TPR;
        wheelDiameter = parameters.wheelDiameter;
        maxForwardSpeed = parameters.maxForwardSpeed;
        hdgCorrectionPIDconsts = parameters.hdgCorrectionPIDconsts;

        claw = new Claw(parameters.clawParameters, hardwareMap);
        imu = new FriendlyIMU(parameters.imuParameters, hardwareMap);
        drive = new MecanumDrive(parameters.driveParameters, hardwareMap);
        arm = new PixelArm(parameters.pixelArmParameters, hardwareMap);
        planeLauncher = new PlaneLauncher(parameters.planeLauncherParameters, hardwareMap);
    }

}
