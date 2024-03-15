package org.firstinspires.ftc.teamcode.fy23.robot.old;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.FriendlyIMUImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.fy23.units.PIDconsts;

/** RobotA represents the competition robot. It contains five subsystems: a {@link MecanumDriveImpl},
 * and a {@link FriendlyIMUImpl}. */
public class VirtualRobot implements AnyRobot {

    // Subsystems - include only and all the subsystems that this robot actually has
    public final MecanumDriveImpl drive;
    public final FriendlyIMUImpl imu;

    /** Ticks per Rotation - 537.7 for the goBILDA 5203-2402-0019 found on the Strafer V5 */
    public final double TPR = 537.7;

    /** Wheel diameter, in centimeters */
    public final double wheelDiameter = 9.6;

    /** Maximum forward speed in centimeters per second */
    public final double maxForwardSpeed = 150; // approximate for the Strafer V5

    /** There's a few different preset things that this can get set to. The default is loaded from
     * a file called "RobotA.pid" that gets saved by RobotAIMUDriveTuner,
     * but you can also use a hard-coded value or disable PID entirely. */
    public final PIDconsts pidConsts;

    /** Default PID constants for the SDK's PID algorithm on individual DcMotorEx devices. Useful
     * for {@link RudimentaryRampToTarget}, perhaps, which
     * uses DcMotorEx.setVelocity() in the RUN_USING_ENCODER runmode. */
    public final PIDconsts sdkMotorPidConsts;

    /** Pass in the hardwareMap that OpMode / LinearOpMode provides. */
    public VirtualRobot(HardwareMap hardwareMap) {
        MecanumDriveImpl.Parameters driveParams = new MecanumDriveImpl.Parameters();

        driveParams.leftFrontName = "front_left_motor";
        driveParams.leftFrontDirection = REVERSE;

        driveParams.rightFrontName = "front_right_motor";
        driveParams.rightFrontDirection = FORWARD;

        driveParams.leftBackName = "back_left_motor";
        driveParams.leftBackDirection = REVERSE;

        driveParams.rightBackName = "back_right_motor";
        driveParams.rightBackDirection = FORWARD;

        drive = new MecanumDriveImpl(driveParams, hardwareMap);

        // TunablePID tuning for this robot - select exactly one
//        pidConsts = new PIDconsts(0.023, 0.00, 0.00); // use the constants I've had the most success with so far
        pidConsts = new PIDconsts(0, 0, 0); // disable PID (and therefore IMU correction)
        { // load from the file that RobotBIMUDriveTuner saved (comment out the entire code block to disable)
            // modified from SensorBNO055IMUCalibration example
//            File file = AppUtil.getInstance().getSettingsFile("VirtualRobot.pid");
//            pidConsts = new PIDconsts(ReadWriteFile.readFile(file));
        }

        sdkMotorPidConsts = new PIDconsts(0.05, 0, 0);

        FriendlyIMUImpl.Parameters imuParams = new FriendlyIMUImpl.Parameters();
        imuParams.present = true;
        imu = new FriendlyIMUImpl(imuParams, hardwareMap);
    }
}
