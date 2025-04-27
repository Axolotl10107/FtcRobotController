package org.firstinspires.ftc.teamcode.framework.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Make telemetry globally accessible.
 * Store a Telemetry into here from anywhere and then retrieve it from anywhere.
 *
 * One big use case for this is telemetry within a subsystem that you're
 * developing. Your OpMode can store the telemetry instance that the OpMode
 * gives it, and then your subsystem can retrieve it. No shenanigans to get the
 * telemetry passed all the way over there!
 *
 * This also seemed to be helpful somehow when working with FTC Dashboard's
 * MultipleTelemetry. */
public class TelemetrySingleton {

    private static Telemetry instance;

    /** Store a telemetry instance. */
    public static void setInstance(Telemetry argInstance) {
        instance = argInstance;
    }

    /** Retrieve the stored telemetry instance. */
    public static Telemetry getInstance() {
        if (instance == null) {
            // "Fail Fast" - have an error *here*, and not some random place later, so we know where it really happened
            throw new NullPointerException();
        } else {
            return instance;
        }
    }

}
