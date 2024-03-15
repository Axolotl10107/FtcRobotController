package org.firstinspires.ftc.teamcode.fy23.robot.old;

import org.firstinspires.ftc.teamcode.fy23.units.DTS;

/** <b>This is no longer necessary! Please use {@link DTS}.normalize() instead.</b> Makes the components of a DTS add to
 * 1 or less, maintaining proportions. This is done by dividing each component by the sum of all 3, and it is necessary
 * because DcMotor.setPower() simply clips its input from -1 to 1, damaging the proportions between the axes.
 * <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">Game Manual 0 article on this</a>*/
public class DTSscaler {
    private double gcd;
    public double scaledTurn; // for telemetry

    public DTS scale(DTS dts) {
        gcd = Math.max(Math.abs(dts.drive) + Math.abs(dts.turn) + Math.abs(dts.strafe), 1);
        scaledTurn = dts.turn / gcd; // for telemetry
        return new DTS(dts.drive / gcd, dts.turn / gcd, dts.strafe / gcd);
    }
}
