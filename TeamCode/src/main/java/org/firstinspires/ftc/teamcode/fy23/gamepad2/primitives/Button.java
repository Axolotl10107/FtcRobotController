package org.firstinspires.ftc.teamcode.fy23.gamepad2.primitives;

/** Interface that represents a button on a gamepad. Multiple implementations are available. Please choose the one that
 * best suits each use case.*/
public interface Button {

    /** Used internally */
    interface BoolLambda {
        boolean get();
    }

    /** When a button should be considered active depends on the implementation and its parameters. */
    boolean isActive();
}
