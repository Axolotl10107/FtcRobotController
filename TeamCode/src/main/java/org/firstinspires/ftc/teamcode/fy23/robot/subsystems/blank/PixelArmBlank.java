package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PixelArm;

/** A blank implementation of {@link PixelArm} that does nothing. */
public class PixelArmBlank implements PixelArm {

    @Override
    public void setPivotAngle(AngleUnit unit, double angle) {

    }

    @Override
    public void setPivotPower(double power) {

    }

    @Override
    public double getPivotPower() {
        return 0;
    }

    @Override
    public void setElevatorDistance(double distance) {

    }

    @Override
    public void setElevatorPower(double power) {

    }

    @Override
    public double getElevatorPower() {
        return 0;
    }

    @Override
    public void update() {

    }

}
