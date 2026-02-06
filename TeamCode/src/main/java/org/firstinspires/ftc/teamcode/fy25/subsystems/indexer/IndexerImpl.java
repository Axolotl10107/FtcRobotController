package org.firstinspires.ftc.teamcode.fy25.subsystems.indexer;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IndexerImpl implements Indexer {

    CRServo servo;
    DcMotorEx encoder;
    double ticksPerRevolution;

    public IndexerImpl(Parameters parameters) {
        servo = parameters.indexerServo;
        encoder = parameters.encoderMotor;
        ticksPerRevolution = parameters.ticksPerRevolution;
    }

    Index selected = Index.A;
    double ticksPerIndex = ticksPerRevolution / 3;

    double targetPos = 0;

    boolean prepped = false;

    @Override
    public void goTo(Index index) {
        if (prepped) {return;}
        int difference = selected.ordinal() - index.ordinal();

        if (difference > 0) {
            targetPos += ticksPerIndex * difference;
        } else if (difference < 0) {
            targetPos += ticksPerIndex * (difference + 3);
        }
    }

    @Override
    public void next() {
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
    }

    @Override
    public void intake() {
        if (prepped) {
            prepped = false;
            targetPos += ticksPerIndex / 4;
        }
    }

    @Override
    public void update() {
        double ticks = Math.floorMod(encoder.getCurrentPosition(), (int) ticksPerRevolution);


        double error = targetPos - (Math.floorMod(encoder.getCurrentPosition(), (int) ticksPerRevolution));

        if (error < 0) {
            error += ticksPerRevolution;
        }

        if (error > 100) {
            servo.setPower(1);
        } else {
            servo.setPower(0);
        }

        int indexNum = (int) Math.round(ticks / ticksPerIndex) % 3;
        selected = Index.values()[indexNum];
    }
}
