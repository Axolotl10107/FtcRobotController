package org.firstinspires.ftc.teamcode.fy23.teletest;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//@Disabled
@TeleOp(name="IMUprinter", group="TeleTest")
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
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        pitch = orientation.thirdAngle;
        roll = orientation.secondAngle;
        yaw = orientation.firstAngle;

        telemetry.addData("pitch", pitch);
        telemetry.addData("roll", roll);
        telemetry.addData("yaw", yaw);
    }
}
