package org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives;

/** Interface that represents an axis on a gamepad. Multiple implementations are available. Please choose the one that
 * best suits each use case.*/
public interface Axis {

    /** Used internally */
    interface DoubleLambda {
        double get();
    }

    /** The value of an axis depends on the implementation and its parameters. */
    double value();
}
