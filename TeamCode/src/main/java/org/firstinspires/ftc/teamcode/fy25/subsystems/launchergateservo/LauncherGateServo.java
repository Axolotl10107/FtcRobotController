package org.firstinspires.ftc.teamcode.fy25.subsystems.launchergateservo;

import com.qualcomm.robotcore.hardware.Servo;

public interface LauncherGateServo {

    enum State {
        OPEN,
        CLOSED
    }

    class Parameters {
        public Parameters(boolean present) {
            this.present = present;
        }
        public final boolean present;
        public Servo device;
        public double power = 0;
    }
    void open();
    void close();
    boolean isOpen();
    void update();

}
