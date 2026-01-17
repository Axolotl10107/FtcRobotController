package org.firstinspires.ftc.teamcode.fy25.subsystems.motorintake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice.BlankCRServo;
import org.firstinspires.ftc.teamcode.framework.subsystems.blankdevice.BlankMotor;

public interface MotorIntake {
    enum State {
        /** Intake running in */
        RUNIN,
        /** Intake running out */
        RUNOUT,
        /** Intake not running */
        NONE
    }

    class Parameters {
        public Parameters(boolean present) {this.present = present;}

        public final boolean present;

        public CRServo motor = new BlankCRServo();

        public double IntakeTPS = 0;
    }

    void spinIn();

    void spinOut();

    void stop();

    void setState(State state);

    void setIntakeVelocity(double velocity);

    double getIntakeVelocity();

    void update();
}
