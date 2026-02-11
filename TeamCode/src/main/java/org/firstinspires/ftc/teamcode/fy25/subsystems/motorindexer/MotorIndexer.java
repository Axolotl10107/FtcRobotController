package org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.Indexer;

public interface MotorIndexer {

    double getPositionError();

    double getOutputPower();

    double getVelocity();

    int getKSFlag();

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
        public DcMotor indexerMotor;
        public TouchSensor limitSwitch;
        public double ticksPerRevolution; // 8192
    }

    double getRd();

    void manualOverride(int direction);

    void unload();

    void resetEncoder();

    void goTo(Indexer.Index index);

    void next();

    void prepIntake();

    void intake();

    double getEncoder();

    Indexer.Index getIndex();

    double getRelative();

    void update();
}
