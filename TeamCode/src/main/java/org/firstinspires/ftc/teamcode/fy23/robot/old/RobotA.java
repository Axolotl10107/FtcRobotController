package org.firstinspires.ftc.teamcode.fy23.robot.old;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.Claw;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.FriendlyIMUImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PixelArmImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.PlaneLauncherImpl;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.ClawImpl;
import org.firstinspires.ftc.teamcode.fy23.units.PIDconsts;

/** RobotA represents the competition robot. It contains five subsystems: a {@link MecanumDriveImpl},
 * a {@link FriendlyIMUImpl}, a {@link PixelArmImpl},
 * a {@link ClawImpl},
 * and a {@link PlaneLauncherImpl}. */
public class RobotA implements AnyRobot {

    // Subsystems - include only and all the subsystems that this robot actually has
    public final MecanumDriveImpl drive;
    public final FriendlyIMUImpl imu;
    public final PixelArmImpl pixelArm;
    public final ClawImpl claw;
    public final PlaneLauncherImpl planeLauncher;

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
        MecanumDriveImpl.Parameters driveParams = new MecanumDriveImpl.Parameters();
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

        drive = new MecanumDriveImpl(driveParams);

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
        imu = new FriendlyIMUImpl(imuParams, hardwareMap);

        PixelArmImpl.Parameters armParams = new PixelArmImpl.Parameters();
        armParams.present = true;
        pixelArm = new PixelArmImpl(armParams, new ElapsedTime());

        Claw.Parameters clawParams = new Claw.Parameters();
        clawParams.present = true;
        claw = new ClawImpl(clawParams);

        PlaneLauncherImpl.Parameters planeLauncherParams = new PlaneLauncherImpl.Parameters();
        planeLauncherParams.present = true;
        planeLauncher = new PlaneLauncherImpl(planeLauncherParams);
    }
}
