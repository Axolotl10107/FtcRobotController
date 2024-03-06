package org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives;

public interface Button {

    interface BoolLambda {
        boolean get();
    }

    boolean isActive();
}
