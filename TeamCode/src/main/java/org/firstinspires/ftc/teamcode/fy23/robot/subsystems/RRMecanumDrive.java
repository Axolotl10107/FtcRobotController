package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.BlankMotor;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;
import org.jetbrains.annotations.NotNull;

import java.util.List;

/** A port of RoadRunner Quickstart's SampleMecanumDrive to our new architecture.
 * Learn more at learnroadrunner.com. */
public interface RRMecanumDrive {

    /** See the RoadRunner Quickstart's DriveConstants class. The default values model a perfect goBILDA Strafer using drive encoders for velocity control. On real robots, some calibration will be needed. */
    class DriveConstants {
        public double TICKS_PER_REV = 537.7;
        public double MAX_RPM = 312;
        public boolean RUN_USING_ENCODER = true;
        public PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
                getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

        public double WHEEL_RADIUS = 1.88976; // inches (converetd from 48mm)
        public double GEAR_RATIO = 1;
        public double TRACK_WIDTH = 16.25; // wheelbase is about 13.125  // TODO: Check against a *stock* Strafer

        public double kV = 1.0 / rpmToVelocity(MAX_RPM);
        public double kA = 0;
        public double kStatic = 0;

        public double MAX_VEL = 50;
        /** The default value here (50 in./sec.) matches the default for MAX_VEL, so it takes 1 second to get to full speed. This may be slow, so make sure to adjust this. */
        public double MAX_ACCEL = 50;
        public double MAX_ANG_VEL = Math.toRadians(60);
        public double MAX_ANG_ACCEL = Math.toRadians(60);

        /** Make sure to adjust this */
        public RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        /** Make sure to adjust this */
        public RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        public double rpmToVelocity(double rpm) {
            return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
        }
        public double getMotorVelocityF(double ticksPerSecond) {
            return 32767 / ticksPerSecond;
        }
    }

    /** Contains motor names and settings - usually part of a set of Robot parameters. For fields without descriptions, see RoadRunner Quickstart's SampleMecanumDrive. */
    class Parameters {
        /** Is this subsystem installed on this robot? */
        public boolean present;

        /** Use the {@link DriveConstants} class included in the RRMecanumDrive interface. */
        public DriveConstants dc;

        /** Ignore this - it will be handled by the Robot class */
        public FriendlyIMU imu;

        /** The motor object on the left front corner of the drivebase, already instantiated */
        public DcMotorEx leftFrontMotor;
        /** The motor object on the right front corner of the drivebase, already instantiated */
        public DcMotorEx rightFrontMotor;
        /** The motor object on the left back corner of the drivebase, already instantiated */
        public DcMotorEx leftBackMotor;
        /** The motor object on the right back corner of the drivebase, already instantiated */
        public DcMotorEx rightBackMotor;

        /** An AccelLimiter object, already instantiated */
        public AccelLimiter accelLimiter;
        /** Whether or not to apply acceleration control. If this is set to <b>false</b>, the accelLimiter parameter
         * does not need to be populated. Defaults to <b>true</b>. */
        public boolean useAccelLimiter = true;
        /** An ElapsedTime or MockElapsedTime object, already instantiated */
        public ElapsedTime stopwatch;

        /** Applies to all motors */
        public DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;

        /** Applies to all motors */
        public DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

        public PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
        public PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

        public double LATERAL_MULTIPLIER = 1;

        public double VX_WEIGHT = 1;
        public double VY_WEIGHT = 1;
        public double OMEGA_WEIGHT = 1;

        public VoltageSensor batteryVoltageSensor;
    }

    TrajectoryVelocityConstraint VEL_CONSTRAINT = new TrajectoryVelocityConstraint() {
        @Override
        public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
            return 0;
        }
    };
    TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = new TrajectoryAccelerationConstraint() {
        @Override
        public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
            return 0;
        }
    };

    /** The leftFront motor, if direct access is needed */
    DcMotorEx leftFront = new BlankMotor();
    /** The rightFront motor, if direct access is needed */
    DcMotorEx rightFront = new BlankMotor();
    /** The leftBack motor, if direct access is needed */
    DcMotorEx leftBack = new BlankMotor();
    /** The rightBack motor, if direct access is needed */
    DcMotorEx rightBack = new BlankMotor();

    /** Apply motor powers from a DTS (Drive-Turn-Strafe).
     * @param dts The DTS to apply. Normalize it before passing it in for desirable behavior. */
    void applyDTS(DTS dts);

    /** The usual DcMotor method, but applied to all four motors.
     * @param runMode The RunMode to set */
    void setMode(DcMotor.RunMode runMode);

    /** The usual DcMotor method, but applied to all four motors.
     * @param behavior The ZeroPowerBehavior to set */
    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior);

    /** Called by robot.update(). You do not need to call this method. */
    void update();

    /** We'll usually use TrajectorySequenceBuilder. Learn more about TrajectoryBuilder at https://learnroadrunner.com/trajectories.html#building-a-trajectory. */
    TrajectoryBuilder trajectoryBuilder(Pose2d startPose);
    /** We'll usually use TrajectorySequenceBuilder. Learn more about TrajectoryBuilder at https://learnroadrunner.com/trajectories.html#building-a-trajectory. */
    TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed);
    /** We'll usually use TrajectorySequenceBuilder. Learn more about TrajectoryBuilder at https://learnroadrunner.com/trajectories.html#building-a-trajectory. */
    TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading);

    /** Learn more at https://learnroadrunner.com/trajectory-sequence.html.*/
    TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose);

    void turnAsync(double angle);
    void turn(double angle);

    void followTrajectoryAsync(Trajectory trajectory);
    void followTrajectory(Trajectory trajectory);

    void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence);
    void followTrajectorySequence(TrajectorySequence trajectorySequence);

    Pose2d getLastError();

    void waitForIdle();

    boolean isBusy();

    void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients);

    void setWeightedDrivePower(Pose2d drivePower);

    @NonNull
    List<Double> getWheelPositions();
    List<Double> getWheelVelocities();

    void setMotorPowers(double v, double v1, double v2, double v3);

    /** This exists only to match SampleMecanumDrive. Attempting to use it will throw an UnsupportedOperationException. Use your robot's FriendlyIMU instead. */
    double getRawExternalHeading();
    /** This exists only to match SampleMecanumDrive. Attempting to use it will throw an UnsupportedOperationException. Use your robot's FriendlyIMU instead. */
    Double getExternalHeadingVelocity();

    TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth);
    TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel);

    // methods from RR's MecanumDrive that are not overridden in SampleMecanumDrive
    void setDrivePower();
    void setDriveSignal();
    void updatePoseEstimate();
    Pose2d getPoseEstimate();
    void setPoseEstimate(Pose2d startPose);
}
