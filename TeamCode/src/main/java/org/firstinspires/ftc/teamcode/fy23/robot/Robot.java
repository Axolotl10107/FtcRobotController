package org.firstinspires.ftc.teamcode.fy23.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.ClawBlank;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.FriendlyIMUBlank;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.MecanumDriveBlank;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.PixelArmBlank;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.PlaneLauncherBlank;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.ClawImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.FriendlyIMUImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PixelArmImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PlaneLauncherImpl;
import org.firstinspires.ftc.teamcode.fy23.units.PIDconsts;
/** Encapsulates all of the components that make up a robot. A simple way to centralize initialization in one place
 * (to avoid duplicated code across OpModes) and make it easier to work with multiple different robots, each of which is
 * defined by a {@link Parameters} class. The Parameters also contain calibration values that tune certain fine
 * behaviors to each robot. An OpMode no longer needs to set up every individual motor and servo. It needs only to
 * create a robot and use the powerful and convenient methods provided by its subsystems. */
public class Robot {

    public static class Parameters {
        double tpr; /** ticks per rotation */
        double wheelDiameter;
        double maxForwardSpeed;
        double driveToStrafeDistCV; // conversion factor from driving distance to equivalent
        // strafing distance, in encoder ticks
        PIDconsts hdgCorrectionPIDconsts; /** used by IMUcorrector */

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

    public final Claw claw;
    public final FriendlyIMU imu;
    public final MecanumDrive drive;
    public final PixelArm arm;
    public final PlaneLauncher planeLauncher;

    public Robot(Parameters parameters, HardwareMap hardwareMap) {
        TPR = parameters.tpr;
        wheelDiameter = parameters.wheelDiameter;
        wheelCircumference = Math.PI * wheelDiameter;
        maxForwardSpeed = parameters.maxForwardSpeed;
        hdgCorrectionPIDconsts = parameters.hdgCorrectionPIDconsts;

        if (parameters.clawParameters.present) {
            claw = new ClawImpl(parameters.clawParameters, hardwareMap);
        } else {
            claw = new ClawBlank();
        }
        // above block and below statements work the same way
        imu = (parameters.imuParameters.present) ? new FriendlyIMUImpl(parameters.imuParameters, hardwareMap) : new FriendlyIMUBlank();
        drive = (parameters.driveParameters.present) ? new MecanumDriveImpl(parameters.driveParameters, hardwareMap) : new MecanumDriveBlank();
        arm = (parameters.pixelArmParameters.present) ? new PixelArmImpl(parameters.pixelArmParameters, hardwareMap) : new PixelArmBlank();
        planeLauncher = (parameters.planeLauncherParameters.present) ? new PlaneLauncherImpl(parameters.planeLauncherParameters, hardwareMap) : new PlaneLauncherBlank();
    }

}
