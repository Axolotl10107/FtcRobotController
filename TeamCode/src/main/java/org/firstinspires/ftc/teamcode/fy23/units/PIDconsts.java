package org.firstinspires.ftc.teamcode.fy23.units;

import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;

import java.util.Arrays;
import java.util.Iterator;

/** Container for PID tuning constants. They cannot be changed after the object is created.
 * Accepted by {@link TunablePID}. */
public class PIDconsts {

    /** proportional */
    public double p;

    /** integral multiplier */
    public double im;

    /** derivative multiplier */
    public double dm;

    public PIDconsts(double argP, double argIM, double argDM) {
        p = argP;
        im = argIM;
        dm = argDM;
    }

    /** Takes a string previously created by the serialize() method. Made with help from
     * <a href="https://stackoverflow.com/questions/7021074/string-delimiter-in-string-split-method">Stack Overflow</a> */
    public PIDconsts(String arg) {
        // and how I'm getting this into the variables seems awful but I'll worry about that later
        Iterator constsIter = Arrays.stream(arg.split(";")).iterator();
        p = Double.parseDouble((String) constsIter.next());
        im = Double.parseDouble((String) constsIter.next());
        dm = Double.parseDouble((String) constsIter.next());
    }

    /** Puts constants in a string that can be written to a file. Made with help from
     * <a href="https://docs.oracle.com/javase/8/docs/api/java/util/Formatter.html#syntax">Oracle docs</a> */
    public String serialize() {
        //A few ways to do this:
        //return String.valueOf(p) + ";" + String.valueOf(im) + ";" + String.valueOf(dm);
        //return Double.toString(p) + ";" + Double.toString(im) + ";" + Double.toString(dm);
            // *Every* object has a *.toString() method.
        return String.format("%f;%f;%f", p, im, dm); // see URL in the JavaDoc comment above
    }
}
