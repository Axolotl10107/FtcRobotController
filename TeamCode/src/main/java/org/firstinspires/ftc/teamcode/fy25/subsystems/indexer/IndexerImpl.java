package org.firstinspires.ftc.teamcode.fy25.subsystems.indexer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class IndexerImpl implements Indexer {

    private final CRServo servo;
    private final DcMotorEx encoder;

    private final TouchSensor limitSwitch;

    private final double ticksPerRevolution = 8192;
    private final double ticksPerIndex = 2730;

    private final double intakeTicks = 1200;

    private static final double kP = 0.00018; // raise until oscillation
    private static final double MAX_POWER = 0.7;
    private double kI = 0.000017;   // raise until oscillation/overshooting
    private double kD = 0.00025;    // increase until no overshooting
    private double integral = 0;
    private double lastError = 0;
    private double minPower = 0.05;       // minimum indexer power
    private double maxIntegral = 5000;    // increase until indexer moves smoothly (test under load as well)
    private double maxRampDistance = 200; // adjust until slowly reaches target

    private Index selected = Index.A;
    private double remainingDelta = 0;
    private double lastEncoderPos;
    private boolean prepping = false;


    public IndexerImpl(Parameters parameters) {
        servo = parameters.indexerServo;
        encoder = parameters.encoderMotor;
        limitSwitch = parameters.limitSwitch;

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastEncoderPos = getEncoder();
    }

    @Override
    public double getRd() {
        return remainingDelta;
    }

    @Override
    public void manualOverride(int direction) {
        remainingDelta = direction * 1200;
        integral = 0;
    }

    @Override
    public void unload() {
        servo.setPower(1);
    }

    @Override
    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void goTo(Index index) {
        int numIndexes = Index.values().length;
        int currentIndex = selected.ordinal();
        int targetIndex = index.ordinal();

        int forwardSteps = targetIndex - currentIndex;
        if (forwardSteps < 0) {
            forwardSteps += numIndexes;
        }

        remainingDelta += forwardSteps * ticksPerIndex;
        selected = index;
    }

    @Override
    public void next() {
        remainingDelta += ticksPerIndex;
        selected = Index.values()[
                (selected.ordinal() + 1) % Index.values().length
                ];
        integral = 0;
    }

    @Override
    public void prepIntake() {
        if (prepping) {return;}
        remainingDelta -= intakeTicks;
        prepping = true;
        integral = 0;
    }

    @Override
    public void intake() {
        if (!prepping) {return;}
        remainingDelta += intakeTicks;
        prepping = false;
        integral = 0;
    }

    @Override
    public double getEncoder() {
        return encoder.getCurrentPosition();
    }

    @Override
    public Indexer.Index getIndex() {
        double relative = ((getEncoder() % ticksPerRevolution) + ticksPerRevolution) % ticksPerRevolution;

        if (relative <= ticksPerIndex) {
            return Indexer.Index.A;
        }

        if (relative <= 2 * ticksPerIndex) {
            return Indexer.Index.B;
        }

        return Indexer.Index.C;
    }

    @Override
    public void update() {
        double currentPos = getEncoder();
        double deltaMoved = currentPos - lastEncoderPos;
        lastEncoderPos = currentPos;

        remainingDelta -= deltaMoved;

//        if (limitSwitch.isPressed() && Math.abs(remainingDelta) < 10) {
//            servo.setPower(0);
//            remainingDelta = 0;
//            lastEncoderPos = getEncoder();
//            integral = 0;
//            lastError = 0;
//            return;
//        }

        double error = remainingDelta;

        if (Math.abs(error) <= 5) {
            servo.setPower(Math.signum(error) * minPower);
            remainingDelta = 0;
            integral = 0;
            lastError = 0;
            return;
        }

        integral += error;
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));

        double derivative = error - lastError;
        lastError = error;

        double power = kP * error + kI * integral + kD * derivative;
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        double absError = Math.abs(error);
        if (absError < maxRampDistance) {
            double scale = absError / maxRampDistance;
            power = Math.signum(power) * Math.max(minPower, Math.abs(power) * scale);
        }

        servo.setPower(-power);
    }


}
