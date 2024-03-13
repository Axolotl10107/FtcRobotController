package org.firstinspires.ftc.teamcode.fy23.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.ClawImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.FriendlyIMUImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PixelArmImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PlaneLauncherImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.units.PIDconsts;

public class Robot {

    public static class Parameters {
        double tpr;
        double wheelDiameter;
        double maxForwardSpeed;
        double driveToStrafeDistCV; // conversion factor from driving distance to equivalent
        // strafing distance, in encoder ticks
        PIDconsts hdgCorrectionPIDconsts;

        Claw.Parameters clawParameters;
        FriendlyIMUImpl.Parameters imuParameters;
        MecanumDriveImpl.Parameters driveParameters;
        PixelArmImpl.Parameters pixelArmParameters;
        PlaneLauncherImpl.Parameters planeLauncherParameters;
    }

    public final double TPR;
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double maxForwardSpeed;
    public final PIDconsts hdgCorrectionPIDconsts;

    public ClawImpl claw;
    public FriendlyIMUImpl imu;
    public MecanumDriveImpl drive;
    public PixelArmImpl arm;
    public PlaneLauncherImpl planeLauncher;

    public Robot(Parameters parameters, HardwareMap hardwareMap) {
        TPR = parameters.tpr;
        wheelDiameter = parameters.wheelDiameter;
        wheelCircumference = Math.PI * wheelDiameter;
        maxForwardSpeed = parameters.maxForwardSpeed;
        hdgCorrectionPIDconsts = parameters.hdgCorrectionPIDconsts;

        claw = new ClawImpl(parameters.clawParameters, hardwareMap);
        imu = new FriendlyIMUImpl(parameters.imuParameters, hardwareMap);
        drive = new MecanumDriveImpl(parameters.driveParameters, hardwareMap);
        arm = new PixelArmImpl(parameters.pixelArmParameters, hardwareMap);
        planeLauncher = new PlaneLauncherImpl(parameters.planeLauncherParameters, hardwareMap);
    }

}
