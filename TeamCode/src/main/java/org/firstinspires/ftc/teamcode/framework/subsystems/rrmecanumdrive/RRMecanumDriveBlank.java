package org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice.BlankMotor;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** A blank implementation of {@link RRMecanumDrive} (and formerly MecanumDrive) that does nothing.
 * Can also be used for any variant of that interface. Never returns null (returns blank objects
 * instead where applicable), so hopefully no null pointers. */
public class RRMecanumDriveBlank implements RRMecanumDrive {

    @Override
    public DcMotorEx getLeftFrontMotor() {
        return new BlankMotor();
    }

    @Override
    public DcMotorEx getRightFrontMotor() {
        return new BlankMotor();
    }

    @Override
    public DcMotorEx getLeftBackMotor() {
        return new BlankMotor();
    }

    @Override
    public DcMotorEx getRightBackMotor() {
        return new BlankMotor();
    }

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
        return new TrajectoryBuilder(startPose, (v, pose2d, pose2d1, pose2d2) -> 0, (v, pose2d, pose2d1, pose2d2) -> 0);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, (v, pose2d, pose2d1, pose2d2) -> 0, (v, pose2d, pose2d1, pose2d2) -> 0);
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, (v, pose2d, pose2d1, pose2d2) -> 0, (v, pose2d, pose2d1, pose2d2) -> 0);
    }

    @Override
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(startPose, (v, pose2d, pose2d1, pose2d2) -> 0, (v, pose2d, pose2d1, pose2d2) -> 0, 0, 0);
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
        return new Pose2d();
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
