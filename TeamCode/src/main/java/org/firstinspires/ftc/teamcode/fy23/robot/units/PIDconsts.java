package org.firstinspires.ftc.teamcode.fy23.robot.units;

public class PIDconsts {
    public double p;  // proportional
    public double im; // integral multiplier
    public double dm; // derivative multiplier

    public PIDconsts(double argP, double argIM, double argDM) {
        p = argP;
        im = argIM;
        dm = argDM;
    }
}
