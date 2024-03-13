package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

public interface PixelArm {

    class Parameters {
        public boolean present;
    }

    void setPivotPower(double power);
    double getPivotPower();

    void setElevatorPower(double power);
    double getElevatorPower();

}
