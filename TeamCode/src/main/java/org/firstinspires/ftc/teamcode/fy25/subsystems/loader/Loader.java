package org.firstinspires.ftc.teamcode.fy25.subsystems.loader;

import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate;
import com.qualcomm.robotcore.hardware.Servo;

public interface Loader {
    enum State {
        LOAD,
        PASS
    }

    class Parameters {
        public Parameters(boolean present) {
            this.present = present;
        }
        public final boolean present;
        public Servo device;
    }

    public void load();
    public void pass();
    public boolean isLoad();
    public void update();
}
