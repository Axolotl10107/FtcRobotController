package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/** A normal implementation of {@link org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU}.
 * The IMU is read each time you call a method. Values are always current (and it's not reading the IMU when you
 * don't need it to). */
public class FriendlyIMUImpl implements org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU {

    private double pitchVel;
    private double rollVel;
    private double yawVel;

    public IMU imu;

    YawPitchRollAngles orientation;

    public FriendlyIMUImpl(Parameters parameters, HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                parameters.logoFacingDirection,
                                parameters.usbFacingDirection
                        )
                )
        );
    }

    private void updateOrientation() {
        orientation = imu.getRobotYawPitchRollAngles();
    }

    private void updateVelocity() {
        AngularVelocity angVel = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        pitchVel = angVel.xRotationRate;
        rollVel = angVel.yRotationRate;
        yawVel = angVel.zRotationRate;
    }

    @Override
    public double pitch() {
        updateOrientation();
        return orientation.getPitch(AngleUnit.DEGREES);
    }

    @Override
    public double pitch(AngleUnit angleUnit) {
        updateOrientation();
        return orientation.getPitch(angleUnit);
    }

    @Override
    public double pitchVel() {
        updateVelocity();
        return pitchVel;
    }

    @Override
    public double pitchVel(AngleUnit angleUnit) {
        updateVelocity();
        return angleUnit.fromDegrees(pitchVel);
    }

    @Override
    public double roll() {
        updateOrientation();
        return orientation.getRoll(AngleUnit.DEGREES);
    }

    @Override
    public double roll(AngleUnit angleUnit) {
        updateOrientation();
        return orientation.getRoll(angleUnit);
    }

    @Override
    public double rollVel() {
        updateVelocity();
        return rollVel;
    }

    @Override
    public double rollVel(AngleUnit angleUnit) {
        updateVelocity();
        return angleUnit.fromDegrees(rollVel);
    }

    @Override
    public double yaw() {
        updateOrientation();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    @Override
    public double yaw(AngleUnit angleUnit) {
        updateOrientation();
        return orientation.getYaw(angleUnit);
    }

    @Override
    public double yawVel() {
        updateVelocity();
        return yawVel;
    }

    @Override
    public double yawVel(AngleUnit angleUnit) {
        updateVelocity();
        return angleUnit.fromDegrees(yawVel);
    }

    @Override
    /** Called by robot.update(). You do not need to call this method. */
    public void update() {

    }

}
