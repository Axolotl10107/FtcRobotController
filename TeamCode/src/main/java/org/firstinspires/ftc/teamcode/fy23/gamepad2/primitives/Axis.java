package org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives;

public interface Axis {

    interface DoubleLambda {
        double get();
    }

    double value();
}
