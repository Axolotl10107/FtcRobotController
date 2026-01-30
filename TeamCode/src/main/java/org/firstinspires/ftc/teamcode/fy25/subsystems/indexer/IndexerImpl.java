package org.firstinspires.ftc.teamcode.fy25.subsystems.indexer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// TODO: ↓↓↓
/** michael help
 * this one scares me
 * it doesn't work and i can't figure out why
 * thanks
 * - sincerely, a very tired man
 */
public class IndexerImpl implements Indexer {

    private final CRServo servo;
    private final DcMotorEx encoder;

    private final double ticksPerRevolution = 8192;
    private final double ticksPerIndex = 2730;

    private final double intakeTicks = 1200;

    private static final double kP = 0.00022;
    private static final double MAX_POWER = 0.7;
    private static final double DEADBAND = 50;

    private Index selected = Index.A;
    private double remainingDelta = 0;
    private double lastEncoderPos;
    private boolean prepping = false;

    public IndexerImpl(Parameters parameters) {
        servo = parameters.indexerServo;
        encoder = parameters.encoderMotor;

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
    // Schrodinger's method
    // it hasn't been tested, therefore it is both functional and not functional
    // shut up i know it doesn't work but let me live in my delusions

    @Override
    public void next() {
        remainingDelta += ticksPerIndex;
        selected = Index.values()[
                (selected.ordinal() + 1) % Index.values().length
                ];
    }

    @Override
    public void prepIntake() {
        if (prepping) {return;}
        remainingDelta -= intakeTicks;
        prepping = true;
    }

    @Override
    public void intake() {
        if (!prepping) {return;}
        remainingDelta += intakeTicks;
        prepping = false;
    }

    @Override
    public double getEncoder() {
        return encoder.getCurrentPosition();
    }
    // hey at least this one works

    @Override
    public Indexer.Index getIndex() {
        double relative = getEncoder() % ticksPerRevolution;

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

        if (Math.abs(remainingDelta) < DEADBAND) {
            servo.setPower(0);
            remainingDelta = 0;
            return;
        }

        double power = remainingDelta * kP;
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        servo.setPower(-power);
    }
}
