package org.firstinspires.ftc.teamcode.framework.units;

import com.acmerobotics.roadrunner.geometry.Vector2d;

/** An immutable vector represented as <b>D</b>rive, <b>T</b>urn, and <b>S</b>trafe axes.
 * Every method of a DTS returns a new DTS with the specified operation applied. */
public class DTS {

    /** Forward / backward movement. */
    public final double drive;

    /** Turning (yaw). */
    public final double turn;

    /** Left / right movement. */
    public final double strafe;

    /** Create a DTS with the specified values set for each component. */
    public DTS(double argDrive, double argTurn, double argStrafe) {
        drive = argDrive;
        turn = argTurn;
        strafe = argStrafe;
    }

    /** Create a DTS with 0 set for every component. */
    public DTS() {
        this(0,0,0);
    }

    /** Negates all 3 components. */
    public DTS negate() {
        return new DTS(-drive, -turn, -strafe);
    }

    /** Adds the components of the given DTS to this DTS.
     * {@param dts} The DTS to add to this one. */
    public DTS add(DTS dts) {
        return new DTS(drive + dts.drive, turn + dts.turn, strafe + dts.strafe);
    }

    /** Multiply all 3 components by the scaling factor.
     * {@param factor} The scaling factor. */
    public DTS scale(double factor) {
        return new DTS(drive * factor, turn * factor, strafe * factor);
    }

    /** Scales everything down (if / as much as) necessary to keep the sum of
     * the 3 components between -1 and 1.
     * You'll usually want to give a normalized DTS to {@link org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDrive},
     * because otherwise a motor will eventually be asked to do 1.5 power, which is impossible. */
    public DTS normalize() {
        double divisor = Math.max((Math.abs(drive) + Math.abs(turn) + Math.abs(strafe)), 1);
        if (divisor != 1) {
            return new DTS(drive / divisor, turn / divisor, strafe / divisor);
        } else {
            return this;
        }
    }

    /** Replace the <b>drive</b> component but leave the other 2 alone.
     * {@param newDrive} The new value of the <b>drive</b> component. */
    public DTS withDrive(double newDrive) {
        return new DTS(newDrive, turn, strafe);
    }

    /** Replace the <b>turn</b> component but leave the other 2 alone.
     * {@param newTurn} The new value of the <b>turn</b> component. */
    public DTS withTurn(double newTurn) {
        return new DTS(drive, newTurn, strafe);
    }

    /** Replace the <b>strafe</b> component but leave the other 2 alone.
     * {@param newStrafe} The new value of the <b>strafe</b> component. */
    public DTS withStrafe(double newStrafe) {
        return new DTS(drive, turn, newStrafe);
    }

    /** Rotate the drive and strafe axes by a value in Radians. Leaves the turn axis unmodified. Perhaps most useful for
     * field-oriented driving, or perhaps driving with independent axes on an "XDrive" base. */
    public DTS rotate(double radians) {
        Vector2d rrVector = new Vector2d(drive, strafe).rotated(radians);
        return new DTS(rrVector.component1(), turn, rrVector.component2());
    }
}
