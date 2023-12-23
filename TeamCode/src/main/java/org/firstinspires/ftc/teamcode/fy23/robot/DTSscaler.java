package org.firstinspires.ftc.teamcode.fy23.robot;

import org.firstinspires.ftc.teamcode.fy23.robot.units.DTS;

public class DTSscaler {
    private double gcd;
    public double scaledTurn; // for telemetry

    public DTS scale(DTS dts) {
        gcd = Math.max(Math.abs(dts.drive) + Math.abs(dts.turn) + Math.abs(dts.strafe), 1);
        scaledTurn = dts.turn / gcd; // for telemetry
        return new DTS(dts.drive / gcd, dts.turn / gcd, dts.strafe / gcd);
    }
}
