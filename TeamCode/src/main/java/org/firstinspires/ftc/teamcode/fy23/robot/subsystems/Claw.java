package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {

    public enum State {
        OPEN,
        CLOSED,
        NONE
    }

    public static class Parameters {
        public boolean present;
    }

    public Claw(Parameters parameters, HardwareMap hardwareMap) {

    }
}
