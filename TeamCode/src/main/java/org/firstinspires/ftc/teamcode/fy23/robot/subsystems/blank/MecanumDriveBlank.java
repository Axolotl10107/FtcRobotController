package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;
import org.jetbrains.annotations.NotNull;

import java.util.List;

/** A blank implementation of {@link MecanumDrive} that does nothing. Can also be used for any variant of that interface. */
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
        return null;
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return null;
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return null;
    }

    @Override
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return null;
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
        return null;
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
        return null;
    }

    @Override
    public List<Double> getWheelVelocities() {
        return null;
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
        return null;
    }

    @Override
    public TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return null;
    }

}
