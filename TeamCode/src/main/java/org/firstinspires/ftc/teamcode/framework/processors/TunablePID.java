package org.firstinspires.ftc.teamcode.framework.processors;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.framework.units.PIDConsts;

/** A simple PID algorithm that allows its constants to be changed on the fly. Useful for tuning
 * them in a TeleOp to instantly see the results of changes. It can also import a {@link PIDConsts}.
 * Implementation based on <a href="https://gm0.org/en/latest/docs/software/concepts/control-loops.html#pid-pseudocode">this pseudocode from GM0.</a>
 * Positive error = positive correction. */
public class TunablePID {

    private double _kP;
    private double _kI;
    private double _maxI;
    private double _kD;

    private double _i; // Integral persists
    private double _lastError = 0;
    private double _lastTime = 0;

    private final ElapsedTime _stopwatch;


    // Actions
    public void clearIntegral() {
        _i = 0;
    }

    public double correctFor(double error) {
        double currentTime = _stopwatch.milliseconds();

        double p = _kP * error;
        _i += _kI * (error * (currentTime - _lastTime));
        _i = Range.clip(_i, -_maxI, _maxI);
        double d = _kD * (error - _lastError) / (currentTime - _lastTime);

        _lastTime = currentTime;
        _lastError = error;

        return p + _i + d;
    }

    public int correctFor(int error) {
        return (int) correctFor((double) error);
    }


    // Constructors
    public TunablePID(double kP, double kI, double maxI, double kD, ElapsedTime stopwatch) {
        _kP = kP;
        _kI = kI;
        _maxI = maxI;
        _kD = kD;
        _stopwatch = stopwatch;
    }

    public TunablePID(double kP, double kI, double maxI, double kD) {
        this(kP, kI, maxI, kD, new ElapsedTime());
    }

    public TunablePID(PIDConsts pidConsts, ElapsedTime stopwatch) {
        this(pidConsts.kP, pidConsts.kI, pidConsts.maxI, pidConsts.kD, stopwatch);
    }

    public TunablePID(PIDConsts pidConsts) {
        this(pidConsts, new ElapsedTime());
    }


    // Getters and setters
    public double kP() {
        return _kP;
    }

    public void setkP(double kP) {
        _kP = kP;
    }

    public double kI() {
        return _kI;
    }

    public void setkI(double kI) {
        _kI = kI;
    }

    public double maxI() {
        return _maxI;
    }

    public void setmaxI(double maxI) {
        _maxI = maxI;
    }

    public double kD() {
        return _kD;
    }

    public void setkD(double kD) {
        _kD = kD;
    }

    public double currentIntegralValue() {
        return _i;
    }

}
