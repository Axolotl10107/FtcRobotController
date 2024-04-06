package org.firstinspires.ftc.teamcode.fy23.units;

public class SimplePowerTpSConverter implements PowerTpSConverter {

    private int TpSAtFullPower;
    private int TpSAtHalfPower;

    public SimplePowerTpSConverter(int TpSAtFullPower, int TpSAtHalfPower) {
        this.TpSAtFullPower = TpSAtFullPower;
        this.TpSAtHalfPower = TpSAtHalfPower;
    }


    @Override
    public int powerToTpS(double power) {
        if (power > 0.5) {
            return (int) (TpSAtFullPower * power);
        } else {
            return (int) (TpSAtHalfPower * (power * 2));
        }
    }

    @Override
    public double TpSToPower(int TpS) {
        if (TpS > TpSAtHalfPower) {
            return (double) TpS / TpSAtFullPower;
        } else {
            return ((double) TpS / TpSAtHalfPower) / 2;
        }
    }
}
