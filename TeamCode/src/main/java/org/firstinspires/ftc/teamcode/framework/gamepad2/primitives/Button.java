package org.firstinspires.ftc.teamcode.framework.gamepad2.primitives;

/** Interface that represents a button on a gamepad. Multiple implementations are available. Choose the one that
 * best suits each use case.*/
public interface Button {

    /** Used internally
     * This is for implementing a lambda expression in Java. See https://docs.oracle.com/javase/tutorial/java/javaOO/lambdaexpressions.html.
     * In our case, <code>() -> {driver.x}</code>, for example, is a lambda expression that returns a boolean.
     * This interface allows a method in a Button implementation to accept that lambda expression. When the implementation
     * calls .get() on it, that expression is evaluated, which polls the actual gamepad at that moment,
     * and the <i>current</i> value of driver.x is returned. */
    interface BoolLambda {
        boolean get();
    }

    /** When a button should be considered active depends on the implementation and its parameters. */
    boolean isActive();
}
