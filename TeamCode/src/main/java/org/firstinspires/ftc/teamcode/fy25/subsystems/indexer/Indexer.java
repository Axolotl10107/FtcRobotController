package org.firstinspires.ftc.teamcode.fy25.subsystems.indexer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public interface Indexer {

    enum State {
        PREP,
        READY,
        TO,
        NEXT
    }
    enum Index {
        A,
        B,
        C
    }

    class Parameters {
        /** Create a Parameters object and provide necessary parameters.
         * @param present Is this subsystem installed on this robot?
         */
        public Parameters(boolean present) {
            this.present = present;
        }

        /** You have already set this in the constructor and cannot set it again. */
        public final boolean present;

        public DcMotorEx encoderMotor;
        public CRServo indexerServo;
        public TouchSensor limitSwitch;
        public double ticksPerRevolution; // 8192
    }

    double getRd();

    void manualOverride(int direction);

    void unload();

    void resetEncoder();

    void goTo(Index index);

    void next();

    void prepIntake();

    void intake();

    double getEncoder();

    Index getIndex();

    void update();

}
