package org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.framework.processors.AccelLimiter;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** A normal implementation of {@link RRMecanumDrive}.
 * Anything lacking a description is from RoadRunner. See learnroadrunner.com to
 * learn more about those things. */
@Config
public class RRMecanumDriveImpl extends MecanumDrive implements RRMecanumDrive {

    public final DcMotorEx leftFront;
    public final DcMotorEx rightFront;
    public final DcMotorEx leftBack;
    public final DcMotorEx rightBack;

    private double lfPowerTarget = 0;
    private double rfPowerTarget = 0;
    private double lbPowerTarget = 0;
    private double rbPowerTarget = 0;

    private final boolean useAccelLimiter;
    private boolean inDTSMode = false;

    private final AccelLimiter accelLimiter;
    private final ElapsedTime stopwatch;
    private final FriendlyIMU imu;

    public static double LATERAL_MULTIPLIER;

    public final double VX_WEIGHT;
    public final double VY_WEIGHT;
    public final double OMEGA_WEIGHT;

    private final TrajectorySequenceRunner trajectorySequenceRunner;

    public final TrajectoryVelocityConstraint VEL_CONSTRAINT;
    public final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT;

    private final double MAX_ANG_VEL;
    private final double MAX_ANG_ACCEL;
    private final double WHEEL_RADIUS;
    private final double GEAR_RATIO;
    private final double TICKS_PER_REV;

    private final List<DcMotorEx> motors;

    private final VoltageSensor batteryVoltageSensor;

    private final List<Integer> lastEncPositions = new ArrayList<>();
    private final List<Integer> lastEncVels = new ArrayList<>();

    public RRMecanumDriveImpl(RRMecanumDrive.Parameters parameters) {
        super(parameters.dc.kV, parameters.dc.kA, parameters.dc.kStatic, parameters.dc.TRACK_WIDTH, parameters.dc.TRACK_WIDTH, parameters.LATERAL_MULTIPLIER);

        // Unpack parameters
        leftFront = parameters.leftFrontMotor;
        rightFront = parameters.rightFrontMotor;
        leftBack = parameters.leftBackMotor;
        rightBack = parameters.rightBackMotor;

        LATERAL_MULTIPLIER = parameters.LATERAL_MULTIPLIER;

        VX_WEIGHT = parameters.VX_WEIGHT;
        VY_WEIGHT = parameters.VY_WEIGHT;
        OMEGA_WEIGHT = parameters.OMEGA_WEIGHT;

        VEL_CONSTRAINT = getVelocityConstraint(parameters.dc.MAX_VEL, parameters.dc.MAX_ANG_VEL, parameters.dc.TRACK_WIDTH);
        ACCEL_CONSTRAINT = getAccelerationConstraint(parameters.dc.MAX_ACCEL);

        MAX_ANG_VEL = parameters.dc.MAX_ANG_VEL;
        MAX_ANG_ACCEL = parameters.dc.MAX_ANG_ACCEL;
        WHEEL_RADIUS = parameters.dc.WHEEL_RADIUS;
        GEAR_RATIO = parameters.dc.GEAR_RATIO;
        TICKS_PER_REV = parameters.dc.TICKS_PER_REV;

        accelLimiter = parameters.accelLimiter;
        useAccelLimiter = parameters.useAccelLimiter;
        stopwatch = parameters.stopwatch;
        imu = parameters.imu;

        batteryVoltageSensor = parameters.batteryVoltageSensor;

        TrajectoryFollower follower = new HolonomicPIDVAFollower(parameters.TRANSLATIONAL_PID, parameters.TRANSLATIONAL_PID, parameters.HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        // SampleMecanumDrive constructs the IMU here, we get it from the Robot instead

        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (leftFront.getMode() == DcMotor.RunMode.RUN_USING_ENCODER && parameters.dc.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, parameters.dc.MOTOR_VELO_PID);
        }

        // A friendly reminder that robot-specific properties like these are set in RobotRoundhouse
//        leftFront.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.FORWARD);
//        leftBack.setDirection(DcMotor.Direction.REVERSE);
//        rightBack.setDirection(DcMotor.Direction.FORWARD);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // If desired, use setLocalizer() to change the localization method:
        // setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, parameters.HEADING_PID, parameters.batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );

        // SampleMecanumDrive checks for RUN_USING_ENCODER in DriveConstants. Please use the existing RunMode field in the MecanumDrive parameters instead.
        try {
            setMode(parameters.runMode);
        } catch (Exception x) {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // if the RunMode wasn't set, pick a default
        }

        try {
            setZeroPowerBehavior(parameters.zeroPowerBehavior);
        } catch (Exception x) {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public DcMotorEx getLeftFrontMotor() {
        return leftFront;
    }

    @Override
    public DcMotorEx getRightFrontMotor() {
        return rightFront;
    }

    @Override
    public DcMotorEx getLeftBackMotor() {
        return leftBack;
    }

    @Override
    public DcMotorEx getRightBackMotor() {
        return rightBack;
    }

    /**
     * Apply motor powers from a DTS (Drive-Turn-Strafe).
     * This implementation will apply acceleration control before it reaches the motors.
     * @param dts The DTS to apply. Normalize it before passing it in for desirable behavior.
     */
    @Override
    public void applyDTS(DTS dts) {
        if (useAccelLimiter) {
            lfPowerTarget = dts.drive - dts.turn + dts.strafe;
            rfPowerTarget = dts.drive + dts.turn - dts.strafe;
            lbPowerTarget = dts.drive - dts.turn - dts.strafe;
            rbPowerTarget = dts.drive + dts.turn + dts.strafe;
            inDTSMode = true;
        } else {
            leftFront.setPower(dts.drive - dts.turn + dts.strafe);
            rightFront.setPower(dts.drive + dts.turn - dts.strafe);
            leftBack.setPower(dts.drive - dts.turn - dts.strafe);
            rightBack.setPower(dts.drive + dts.turn + dts.strafe);
        }
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    @Override
    public void update() {
        if (inDTSMode) {
            tUpdate();
        }
    }

    // for when applyDTS() is being used - called by OpMode
    private void tUpdate() {
        double currentLF = leftFront.getPower();
        double currentRF = rightFront.getPower();
        double currentLB = leftBack.getPower();
        double currentRB = rightBack.getPower();
        double requestedDeltaLF = lfPowerTarget - currentLF;
        double requestedDeltaRF = rfPowerTarget - currentRF;
        double requestedDeltaLB = lbPowerTarget - currentLB;
        double requestedDeltaRB = rbPowerTarget - currentRB;
        List<Double> requestList = Arrays.asList(requestedDeltaLF, requestedDeltaRF, requestedDeltaLB, requestedDeltaRB);

        List<Double> returnList = accelLimiter.requestDeltaVelOnN(requestList, stopwatch.seconds());
        leftFront.setPower(currentLF + returnList.get(0));
        rightFront.setPower(currentRF + returnList.get(1));
        leftBack.setPower(currentLB + returnList.get(2));
        rightBack.setPower(currentRB + returnList.get(3));
    }

    // for when followTrajectory(Sequence) is being used - called by waitForIdle()
    private void rrUpdate() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    @Override
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    @Override
    public void turnAsync(double angle) {
        inDTSMode = false;
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    @Override
    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    @Override
    public void followTrajectoryAsync(Trajectory trajectory) {
        inDTSMode = false;
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    @Override
    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    @Override
    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        inDTSMode = false;
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    @Override
    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    @Override
    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    @Override
    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            rrUpdate();
    }

    @Override
    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    @Override
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    @Override
    public void setWeightedDrivePower(Pose2d drivePower) {
        inDTSMode = false;
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        inDTSMode = false;
        leftFront.setPower(v);
        leftBack.setPower(v1);
        rightBack.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return Math.toRadians(imu.yaw());
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return imu.yawVel();
    }

    @Override
    public TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    @Override
    public TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    // The methods below come from the superclass (MecanumDrive) and were not overridden by SampleMecanumDrive.
    // The interface makes them available, so these callthroughs must exist.
    @Override
    public void setDrivePower() {
        inDTSMode = false;
        super.setDrivePower(new Pose2d());
    }

    @Override
    public void setDriveSignal() {
        inDTSMode = false;
        super.setDriveSignal(new DriveSignal());
    }

    @Override
    public void setDrivePower(@NotNull Pose2d drivePower) {
        inDTSMode = false;
        super.setDrivePower(drivePower);
    }

    @Override
    public void setDriveSignal(@NotNull DriveSignal driveSignal) {
        inDTSMode = false;
        super.setDriveSignal(driveSignal);
    }
}
