package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.apache.commons.math3.analysis.function.Min;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** A blank implementation of {@link MecanumDrive} that does nothing. Can also be
 * used for any variant of that interface. Does not return null, returns blank
 * objects, so hopefully no null pointers. */
public class MecanumDriveBlank implements MecanumDrive, RRMecanumDrive {

    @Override
    public void applyDTS(DTS dts) {

    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {

    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {

    }

    @Override
    public void update() {

    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, new TrajectoryVelocityConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 0;
            }
        }, new TrajectoryAccelerationConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 0;
            }
        });
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, new TrajectoryVelocityConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 0;
            }
        }, new TrajectoryAccelerationConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 0;
            }
        });
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, new TrajectoryVelocityConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 0;
            }
        }, new TrajectoryAccelerationConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 0;
            }
        });
    }

    @Override
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(startPose, new TrajectoryVelocityConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 0;
            }
        }, new TrajectoryAccelerationConstraint() {
            @Override
            public double get(double v, @NotNull Pose2d pose2d, @NotNull Pose2d pose2d1, @NotNull Pose2d pose2d2) {
                return 0;
            }
        }, 0, 0);
    }

    @Override
    public void turnAsync(double angle) {

    }

    @Override
    public void turn(double angle) {

    }

    @Override
    public void followTrajectoryAsync(Trajectory trajectory) {

    }

    @Override
    public void followTrajectory(Trajectory trajectory) {

    }

    @Override
    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {

    }

    @Override
    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {

    }

    @Override
    public Pose2d getLastError() {
        return new Pose2d();
    }

    @Override
    public void waitForIdle() {

    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {

    }

    @Override
    public void setWeightedDrivePower(Pose2d drivePower) {

    }

    @NonNull
    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return new ArrayList<>();
    }

    @Override
    public List<Double> getWheelVelocities() {
        return new ArrayList<>();
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {

    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return 0.0;
    }

    @Override
    public TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(new AngularVelocityConstraint(0), new MecanumVelocityConstraint(0, 0)));
    }

    @Override
    public TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(0);
    }

    @Override
    public void setDrivePower() {

    }

    @Override
    public void setDriveSignal() {

    }

    @Override
    public void updatePoseEstimate() {

    }

    @Override
    public Pose2d getPoseEstimate() {
        return null;
    }

    @Override
    public void setPoseEstimate(Pose2d startPose) {

    }

//    public void setDrivePower(Pose2d drivePower) {
//
//    }
//
//    public void setDriveSignal(DriveSignal driveSignal) {
//
//    }

}
