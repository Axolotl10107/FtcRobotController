package org.firstinspires.ftc.teamcode.framework.ctlpad.primitives;

/** Interface that represents an axis on a gamepad. Multiple implementations are available. Choose the one that
 * best suits each use case.*/
public interface Axis {

    /** Used internally.
     * This is for implementing a lambda expression in Java. See <a href="https://docs.oracle.com/javase/tutorial/java/javaOO/lambdaexpressions.html">Oracle tutorial</a>.
     * In our case, <code>() -> {driver.rightTrigger}</code>, for example, is a lambda expression that returns a double.
     * This interface allows a method in an Axis implementation to accept that lambda expression. When the implementation
     * calls .get() on it, that expression is evaluated, which polls the actual gamepad at that moment,
     * and the <i>current</i> value of driver.rightTrigger is returned. */
    interface DoubleLambda {
        double get();
    }

    /** The value of an axis depends on the implementation and its parameters. */
    double value();
}
