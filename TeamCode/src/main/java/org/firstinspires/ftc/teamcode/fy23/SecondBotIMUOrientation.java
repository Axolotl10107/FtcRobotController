package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class SecondBotIMUOrientation implements ImuOrientationOnRobot {
    @Override
    public Quaternion imuCoordinateSystemOrientationFromPerspectiveOfRobot() {
        return null;
    }

    @Override
    public Quaternion imuRotationOffset() {
        return null;
    }

    @Override
    public Quaternion angularVelocityTransform() {
        return null;
    }
}
