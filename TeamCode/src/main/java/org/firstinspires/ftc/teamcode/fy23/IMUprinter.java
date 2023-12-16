package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMUprinter extends OpMode {
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;

    Orientation orientation;
    float pitch;
    float roll;
    float yaw;

    @Override
    public void init() {
        parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        pitch = orientation.firstAngle;
        roll = orientation.secondAngle;
        yaw = orientation.thirdAngle;

        telemetry.addData("pitch", pitch);
        telemetry.addData("roll", roll);
        telemetry.addData("yaw", yaw);
    }
}
