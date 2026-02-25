package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherangle;

import static org.firstinspires.ftc.teamcode.fy25.subsystems.launcherangle.LauncherAngleImpl.AngleTuning.offsetRight;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.launcherangle.LauncherAngleImpl.AngleTuning.scalar;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.launcherangle.LauncherAngleImpl.AngleTuning.upperBound;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.launcherangle.LauncherAngleImpl.AngleTuning.offsetLeft;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.launcherangle.LauncherAngleImpl.AngleTuning.angleTest;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherAngleImpl implements LauncherAngle {
    private final Servo servoRight;
    private final Servo servoLeft;

    @Config
    public static class AngleTuning {
        public static final double upperBound = .21;
        public static final double offsetLeft = 0;
        public static final double offsetRight = 0;
        public static final double scalar = 360;
        public static final double angleTest = 0;
    }

    public LauncherAngleImpl(Parameters parameters) {
        boolean present = parameters.present;
        servoRight = parameters.servoRight;
        servoLeft = parameters.servoLeft;
    }

    private double posToAngle(double pos) {
        return pos * scalar;
    }

    private double angleToPos(double angle) {
        return angle / scalar;
    }

    private double constrainPos(double pos) {
        if (pos > upperBound) {
            return upperBound;
        }
        return pos;
    }

    @Override
    public double getPositionLeft() {
        return servoLeft.getPosition() + offsetLeft;
    }

    @Override
    public double getPositionRight() {
        return servoRight.getPosition() + offsetRight;
    }

    @Override
    public void setPosition(double position) {
        servoRight.setPosition(constrainPos(position + offsetRight));
        servoLeft.setPosition(constrainPos(position + offsetLeft));
    }

    @Override
    public double getAngle() {return posToAngle((getPositionLeft() + getPositionRight()) / 2);}

    @Override
    public void setAngle(double angle) {setPosition(angleToPos(angle));}

    @Override
    public void testAngle() {setAngle(angleTest);}
}
