package org.firstinspires.ftc.teamcode.fy25.subsystems.indexer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
        public double ticksPerRevolution; // 8192
    }

    void goTo(Index index);

    void next();

    void prepIntake(Index index);

    void intake();

    void update();

}
