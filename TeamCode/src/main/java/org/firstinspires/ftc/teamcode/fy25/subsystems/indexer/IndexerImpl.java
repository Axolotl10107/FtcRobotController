package org.firstinspires.ftc.teamcode.fy25.subsystems.indexer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IndexerImpl implements Indexer {

    private final CRServo servo;
    private final DcMotorEx encoder;

    private final double ticksPerRevolution = 8192;
    private final double ticksPerIndex = 2730;

    private static final double kP = 0.0002;
    private static final double MAX_POWER = 0.7;
    private static final double DEADBAND = 20;

    private double targetPos = 0;
    private Index selected = Index.A;

    public IndexerImpl(Parameters parameters) {
        servo = parameters.indexerServo;
        encoder = parameters.encoderMotor;

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void testServo() {
        servo.setPower(-1);
    }

    @Override
    public void goTo(Index index) {
        int numIndexes = Index.values().length;

        int currentIndex =
                Math.floorMod(
                        (int) Math.round(encoder.getCurrentPosition() / ticksPerIndex),
                        numIndexes
                );

        int targetIndex = index.ordinal();

        int forwardSteps = targetIndex - currentIndex;
        if (forwardSteps < 0) {
            forwardSteps += numIndexes;
        }

        targetPos = encoder.getCurrentPosition() + (forwardSteps * ticksPerIndex);
        selected = index;
    }

    @Override
    public void next() {
        targetPos = encoder.getCurrentPosition() + ticksPerIndex;

        selected = Index.values()[
                (selected.ordinal() + 1) % Index.values().length
                ];
    }

    @Override
    public void prepIntake(Index index) {
        goTo(index);
        targetPos -= ticksPerIndex / 4.0;
    }

    @Override
    public void intake() {
        targetPos += ticksPerIndex / 4.0;
    }

    @Override
    public double getEncoder() {
        return encoder.getCurrentPosition();
    }

    @Override
    public double getTarget() {
        return targetPos;
    }

    @Override
    public void update() {
        double current = encoder.getCurrentPosition();
        double error = targetPos - current;

        if (Math.abs(error) < DEADBAND) {
            servo.setPower(0);
            return;
        }

        double power = error * kP;

        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        servo.setPower(-power);
    }
}
