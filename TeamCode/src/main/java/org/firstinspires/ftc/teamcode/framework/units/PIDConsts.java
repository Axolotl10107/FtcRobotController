package org.firstinspires.ftc.teamcode.framework.units;

import org.firstinspires.ftc.teamcode.framework.processors.TunablePID;

import java.util.Arrays;
import java.util.Iterator;

/** Container for PID tuning constants. They cannot be changed after the object is created.
 * Accepted by {@link TunablePID}. */
public class PIDConsts {

    /** proportional */
    public final double kP;

    /** integral multiplier */
    public final double kI;

    /** maximum integral value */
    public final double maxI;

    /** derivative multiplier */
    public final double kD;

    public PIDConsts(double kP, double kI, double maxI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.maxI = maxI;
        this.kD = kD;
    }


    /** Puts constants in a string. This is useful for writing them to a file. Made with help from
     * <a href="https://docs.oracle.com/javase/8/docs/api/java/util/Formatter.html#syntax">Oracle docs</a>. */
    public String serialize() {
        //A few ways to do this:
        //return String.valueOf(p) + ";" + String.valueOf(im) + ";" + String.valueOf(dm);
        //return Double.toString(p) + ";" + Double.toString(im) + ";" + Double.toString(dm);
        // *Every* object has a *.toString() method.
        return String.format("%f;%f;%f", kP, kI, maxI, kD); // see URL in the JavaDoc comment above
    }

    /** Takes a string previously created by the serialize() method. This is useful for reading PID constants from a file.
     * Made with help from <a href="https://stackoverflow.com/questions/7021074/string-delimiter-in-string-split-method">Stack Overflow</a>. */
    public PIDConsts(String arg) {
        Iterator constsIter = Arrays.stream(arg.split(";")).iterator();
        kP = Double.parseDouble((String) constsIter.next());
        kI = Double.parseDouble((String) constsIter.next());
        maxI = Double.parseDouble((String) constsIter.next());
        kD = Double.parseDouble((String) constsIter.next());
    }


    public PIDConsts withkP(double newkP) {
        return new PIDConsts(newkP, kI, maxI, kD);
    }

    public PIDConsts withkI(double newkI) {
        return new PIDConsts(kP, newkI, maxI, kD);
    }

    public PIDConsts withmaxI(double newmaxI) {
        return new PIDConsts(kP, kI, newmaxI, kD);
    }

    public PIDConsts withkD(double newkD) {
        return new PIDConsts(kP, kI, maxI, newkD);
    }
}
