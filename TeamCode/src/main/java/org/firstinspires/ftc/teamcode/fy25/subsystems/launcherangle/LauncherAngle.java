package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherangle;

import com.qualcomm.robotcore.hardware.Servo;

public interface LauncherAngle {

    class Parameters {
        public Parameters(boolean present) {
            this.present = present;
        }
        public final boolean present;
        public Servo servoRight;
        public Servo servoLeft;
    }

    double getPositionLeft();

    double getPositionRight();

    void setPosition(double position);

    double getAngle();
    void setAngle(double angle);

    void testAngle();
}
