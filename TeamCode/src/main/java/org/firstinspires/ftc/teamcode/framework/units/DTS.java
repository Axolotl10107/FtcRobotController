package org.firstinspires.ftc.teamcode.framework.units;

import com.acmerobotics.roadrunner.geometry.Vector2d;

/** An immutable vector represented as <b>D</b>rive, <b>T</b>urn, and <b>S</b>trafe axes. */
public class DTS {

    public final double drive;
    public final double turn;
    public final double strafe;

    public DTS(double argDrive, double argTurn, double argStrafe) {
        drive = argDrive;
        turn = argTurn;
        strafe = argStrafe;
    }

    public DTS() {
        this(0,0,0);
    }

    public DTS negate() {
        return new DTS(-drive, -turn, -strafe);
    }

    public DTS add(DTS dts) {
        return new DTS(drive + dts.drive, turn + dts.turn, strafe + dts.strafe);
    }

    /** also known as multiplication */
    public DTS scale(double factor) {
        return new DTS(drive * factor, turn * factor, strafe * factor);
    }

    /** Refactor the DTS to be within a normal range, keeping the sum of the axes between -1 and 1 in this case, while
     * maintaining the proportions between the axes. In other words, scales everything down so that the sum of the axes
     * is between -1 and 1. */
    public DTS normalize() {
        double divisor = Math.max((Math.abs(drive) + Math.abs(turn) + Math.abs(strafe)), 1);
        if (divisor != 1) {
            return new DTS(drive / divisor, turn / divisor, strafe / divisor);
        } else {
            return this;
        }
    }

    public DTS withDrive(double newDrive) {
        return new DTS(newDrive, turn, strafe);
    }

    public DTS withTurn(double newTurn) {
        return new DTS(drive, newTurn, strafe);
    }

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
