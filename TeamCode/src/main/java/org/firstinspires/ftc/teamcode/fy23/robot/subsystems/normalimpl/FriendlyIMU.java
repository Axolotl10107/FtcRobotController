package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FriendlyIMU {

    public static class Parameters {
        public boolean present;
    }

    private double pitch;
    private double roll;
    private double yaw;

    public BNO055IMU imu; //our control hubs should have this type
    private BNO055IMU.Parameters imuParams; //stores configuration stuff for the IMU

    Orientation orientation;

    public FriendlyIMU(Parameters parameters, HardwareMap hardwareMap) {
        imuParams = new BNO055IMU.Parameters();
        imuParams.calibrationDataFile = "BNO055IMUCalibration.json";
        //see "SensorBNO055IMUCalibration" example

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);
    }

    private void updateOrientation() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        pitch = orientation.firstAngle;
        roll = orientation.secondAngle;
        yaw = orientation.thirdAngle;
    }


    public double pitch() {
        updateOrientation();
        return pitch;
    }

    public double roll() {
        updateOrientation();
        return roll;
    }

    public double yaw() {
        updateOrientation();
        return yaw;
    }
}
