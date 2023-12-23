package org.firstinspires.ftc.teamcode.old_meddev;

public class RampMath {
    private final double target_;
    private double current_ = 0;
    private double lastTime_;
    private double rate = 1;
    public static final double TPR = 100;//Encoder Ticks per Revolution - May vary by motor, make sure to set accordingly for yours!

    //Constructor - must be called to instantiate class and initialize fields
    RampMath(final double target) {
        target_ = target;
    }

    public void init(double runtime) {
        lastTime_ = runtime;
    }

    public double rampCalc(double runtime) {
        //TODO: Eventually put math here.
        return 0.0;
    }
}

/*
a = m/s^2
a = delta(s)/t

//ramp.setTarget(0.8);
RampMath rampMath = new RampMath(0.8);

while (opModeIsActive()) {
    //other stuff
    double power = ramp.update(current, target, runtimeMillis());
    leftFront.setPower(power)
        }
*/