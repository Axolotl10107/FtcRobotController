package org.firstinspires.ftc.teamcode.fy23.robot.old;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PixelArm;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PlaneLauncher;
import org.firstinspires.ftc.teamcode.fy23.robot.units.PIDconsts;

/** RobotA represents the competition robot. It contains five subsystems: a {@link MecanumDrive},
 * a {@link FriendlyIMU}, a {@link PixelArm},
 * a {@link org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw},
 * and a {@link PlaneLauncher}. */
public class RobotA implements AnyRobot {

    // Subsystems - include only and all the subsystems that this robot actually has
    public final MecanumDrive drive;
    public final FriendlyIMU imu;
    public final PixelArm pixelArm;
    public final Claw claw;
    public final PlaneLauncher planeLauncher;

    /** Ticks per Rotation - 537.7 for the goBILDA 5203-2402-0019 found on the Strafer V5 */
    public final double TPR = 537.7;

    /** Wheel diameter, in centimeters */
    public final double wheelDiameter = 9.6;
    public final double wheelCircumference = wheelDiameter * Math.PI;

    /** Maximum forward speed in centimeters per second */
    public final double maxForwardSpeed = 150;

    /** There's a few different preset things that this can get set to. The default is loaded from
     * a file called "RobotA.pid" that gets saved by RobotAIMUDriveTuner,
     * but you can also use a hard-coded value or disable PID entirely. */
    public final PIDconsts pidConsts;

    /** Default PID constants for the SDK's PID algorithm on individual DcMotorEx devices. Useful
     * for {@link RudimentaryRampToTarget}, perhaps, which
     * uses DcMotorEx.setVelocity() in the RUN_USING_ENCODER runmode. */
    public final PIDconsts sdkMotorPidConsts;

    /** Pass in the hardwareMap that OpMode / LinearOpMode provides. */
    public RobotA(HardwareMap hardwareMap) {
        MecanumDrive.Parameters driveParams = new MecanumDrive.Parameters();

        driveParams.leftFrontName = "leftFront";
        driveParams.leftFrontDirection = REVERSE;

        driveParams.rightFrontName = "rightFront";
        driveParams.rightFrontDirection = FORWARD;

        driveParams.leftBackName = "leftBack";
        driveParams.leftBackDirection = REVERSE;

        driveParams.rightBackName = "rightBack";
        driveParams.rightBackDirection = FORWARD;

        drive = new MecanumDrive(driveParams, hardwareMap);

        // TunablePID tuning for this robot - select exactly one
        pidConsts = new PIDconsts(0.023, 0.00, 0.00); // use the constants I've had the most success with so far
//        pidConsts = new PIDconsts(0, 0, 0); // disable PID (and therefore IMU correction)
        { // load from the file that RobotBIMUDriveTuner saved (comment out the entire code block to disable)
            // modified from SensorBNO055IMUCalibration example
//            File file = AppUtil.getInstance().getSettingsFile("RobotA.pid");
//            pidConsts = new PIDconsts(ReadWriteFile.readFile(file));
        }

        sdkMotorPidConsts = new PIDconsts(0.05, 0, 0);

        FriendlyIMU.Parameters imuParams = new FriendlyIMU.Parameters();
        imuParams.present = true;
        imu = new FriendlyIMU(imuParams, hardwareMap);

        PixelArm.Parameters armParams = new PixelArm.Parameters();
        armParams.present = true;
        pixelArm = new PixelArm(armParams, hardwareMap);

        Claw.Parameters clawParams = new Claw.Parameters();
        clawParams.present = true;
        claw = new Claw(clawParams, hardwareMap);

        PlaneLauncher.Parameters planeLauncherParams = new PlaneLauncher.Parameters();
        planeLauncherParams.present = true;
        planeLauncher = new PlaneLauncher(planeLauncherParams, hardwareMap);
    }
}
