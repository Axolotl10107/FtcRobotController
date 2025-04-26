package org.firstinspires.ftc.teamcode.framework.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetrySingleton {

    private static Telemetry instance;

    public static void setInstance(Telemetry argInstance) {
        instance = argInstance;
    }

    public static Telemetry getInstance() {
        if (instance == null) {
            // "Fail Fast" - have an error *here*, and not some random place later, so we know where it really happened
            throw new NullPointerException();
        } else {
            return instance;
        }
    }

}
