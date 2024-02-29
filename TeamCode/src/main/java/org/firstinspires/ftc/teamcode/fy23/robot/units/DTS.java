package org.firstinspires.ftc.teamcode.fy23.robot.units;

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

    public DTS negate() {
        return new DTS(-drive, -turn, -strafe);
    }

    public DTS add(DTS dts) {
        return new DTS(drive + dts.drive, turn + dts.turn, strafe + dts.strafe);
    }

    public DTS scale(double factor) {
        return new DTS(drive * factor, turn * factor, strafe * factor);
    }

    public DTS normalize() {
        double divisor = Math.max((Math.abs(drive) + Math.abs(turn) + Math.abs(strafe)), 1);
        if (divisor != 1) {
            return new DTS(drive / divisor, turn / divisor, strafe / divisor);
        } else {
            return this;
        }
    }

    public DTS withDrive(double argDrive) {
        return new DTS(argDrive, turn, strafe);
    }

    public DTS withTurn(double argTurn) {
        return new DTS(drive, argTurn, strafe);
    }

    public DTS withStrafe(double argStrafe) {
        return new DTS(drive, turn, argStrafe);
    }
}
