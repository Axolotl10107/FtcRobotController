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

    private static final double kP = 0.00022; // raise until oscillation
    private static final double MAX_POWER = 0.7;
    private double kI = 0.00005;   // raise until oscillation/overshooting
    private double kD = 0.00005;    // increase until no overshooting
    private double integral = 0;
    private double lastError = 0;
    private double minPower = 0.05;       // minimum indexer power
    private double maxIntegral = 5000;    // increase until indexer moves smoothly (test under load as well)
    private double maxRampDistance = 200; // adjust until slowly reaches target

    private Index selected = Index.A;
    private double remainingDelta = 0;
    private double lastEncoderPos;
    private boolean prepping = false;

    CRServo servo;
    DcMotorEx encoder;
    double ticksPerRevolution;

    public IndexerImpl(Parameters parameters) {
        servo = parameters.indexerServo;
        encoder = parameters.encoderMotor;
        ticksPerRevolution = parameters.ticksPerRevolution;
        limitSwitch = parameters.limitSwitch;

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastEncoderPos = getEncoder();
    }

    Index selected = Index.A;
    double ticksPerIndex = ticksPerRevolution / 3;
    @Override
    public double getRd() {
        return remainingDelta;
    }

    double targetPos = 0;
    @Override
    public void manualOverride(int direction) {
        remainingDelta = direction * 1200;
        integral = 0;
    }

    boolean prepped = false;
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
        if (prepped) {return;}
        int difference = selected.ordinal() - index.ordinal();
        int numIndexes = Index.values().length;
        int currentIndex = selected.ordinal();
        int targetIndex = index.ordinal();

        if (difference > 0) {
            targetPos += ticksPerIndex * difference;
        } else if (difference < 0) {
            targetPos += ticksPerIndex * (difference + 3);
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
        if (prepped) {return;}
        targetPos += ticksPerIndex;
        targetPos %= ticksPerRevolution;
    }

    @Override
    public void prepIntake(Index index) {
        if (!prepped) {
            goTo(index);
            targetPos -= ticksPerIndex / 4;
            prepped = true;
        }
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
        if (prepped) {
            prepped = false;
            targetPos += ticksPerIndex / 4;
        }
    }

    @Override
    public void update() {
        double ticks = Math.floorMod(encoder.getCurrentPosition(), (int) ticksPerRevolution);

        double currentPos = getEncoder();
        double deltaMoved = currentPos - lastEncoderPos;
        lastEncoderPos = currentPos;

        double error = targetPos - (Math.floorMod(encoder.getCurrentPosition(), (int) ticksPerRevolution));
        remainingDelta -= deltaMoved;

        if (error < 0) {
            error += ticksPerRevolution;
        }

        if (error > 100) {
            servo.setPower(1);
        } else {
        if (limitSwitch.isPressed() && Math.abs(remainingDelta) < 10) {
            servo.setPower(0);
            remainingDelta = 0;
            lastEncoderPos = getEncoder();
            integral = 0;
            lastError = 0;
            return;
        }

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

        int indexNum = (int) Math.round(ticks / ticksPerIndex) % 3;
        selected = Index.values()[indexNum];
        servo.setPower(-power);
    }


}
