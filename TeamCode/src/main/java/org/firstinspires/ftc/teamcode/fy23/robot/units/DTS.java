package org.firstinspires.ftc.teamcode.fy23.robot.units;

/** A vector represented as <b>D</b>rive, <b>T</b>urn, and <b>S</b>trafe axes. They cannot be changed
 * after the object is created. */
public class DTS {

    public double drive = 0;
    public double turn = 0;
    public double strafe = 0;

    public DTS(double argDrive, double argTurn, double argStrafe) {
        drive = argDrive;
        turn = argTurn;
        strafe = argStrafe;
    }
}
