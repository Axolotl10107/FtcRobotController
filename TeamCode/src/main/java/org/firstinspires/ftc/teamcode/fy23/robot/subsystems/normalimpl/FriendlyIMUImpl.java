package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.*;

/** A normal implementation of {@link org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU}.
 * <b>This class has an open task:</b> Robot and Subsystems / Remake FriendlyIMUImpl */
public class FriendlyIMUImpl implements org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU {

    private double pitch;
    private double pitchVel;

    private double roll;
    private double rollVel;

    private double yaw;
    private double yawVel;

    public BNO055IMU imu; //our control hubs should have this type
    private BNO055IMU.Parameters imuParams; //stores configuration stuff for the IMU

    Orientation orientation;

    public FriendlyIMUImpl(org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU.Parameters parameters, HardwareMap hardwareMap) {
        imuParams = new BNO055IMU.Parameters();
        imuParams.calibrationDataFile = "BNO055IMUCalibration.json";
        //see "SensorBNO055IMUCalibration" example

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);
    }

    private void updateOrientation() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        pitch = orientation.thirdAngle;
        roll = orientation.secondAngle;
        yaw = orientation.firstAngle;
    }

    private void updateVelocity() {
        AngularVelocity angVel = imu.getAngularVelocity();
        pitchVel = angVel.xRotationRate;
        rollVel = angVel.yRotationRate;
        yawVel = angVel.zRotationRate;
    }

    @Override
    public double pitch() {
        updateOrientation();
        return pitch;
    }

    @Override
    public double pitch(AngleUnit angleUnit) {
        updateOrientation();
        return angleUnit.fromDegrees(pitch);
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
        return roll;
    }

    @Override
    public double roll(AngleUnit angleUnit) {
        updateOrientation();
        return angleUnit.fromDegrees(roll);
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
        return yaw;
    }

    @Override
    public double yaw(AngleUnit angleUnit) {
        updateOrientation();
        return angleUnit.fromDegrees(yaw);
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
