package org.firstinspires.ftc.teamcode.fy25.subsystems.indexer;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.IndexerImpl.IndexerTuning.MAX_POWER;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.IndexerImpl.IndexerTuning.kD;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.IndexerImpl.IndexerTuning.kI;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.IndexerImpl.IndexerTuning.kP;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.IndexerImpl.IndexerTuning.kS;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.IndexerImpl.IndexerTuning.kSCutoff;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.IndexerImpl.IndexerTuning.maxIntegral;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.IndexerImpl.IndexerTuning.maxRampDistance;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.IndexerImpl.IndexerTuning.minPower;

import com.acmerobotics.dashboard.config.Config;
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

    private double integral = 0;
    private double lastError = 0;
    private Index selected = Index.A;
    private double remainingDelta = 0;
    private double lastEncoderPos;
    private boolean prepping = false;

    private double positionError = 0;
    private double outputPower = 0;
    private double velocity = 0;
    private boolean kSActive = false;

    @Config
    public static class IndexerTuning {
        public static double MAX_POWER = 1;
        public static double kD = 0.00005;
        public static double kI = 0.000035;
        public static double kP = 0.000079;
        public static double kS = 0.0006;
        public static double kSCutoff = 1600;
        public static double maxIntegral = 950;
        public static double maxRampDistance = 0;
        public static double minPower = 0;

    }


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
//        double pos = getEncoder();
//        if (pos > 0) {
//            pos -= Math.floor(pos / ticksPerRevolution) * ticksPerRevolution;
//        }
//
//        if (pos < 0) {
//            pos -= Math.ceil(pos / ticksPerRevolution) * ticksPerRevolution;
//        }
//
//        if (pos < 300 && pos > -300 ||
//            pos < ticksPerRevolution + 300 && pos > ticksPerRevolution - 300) {
//            return Index.A;
//        } else if (pos < ticksPerIndex + 300 && pos > ticksPerIndex - 300) {
//            return Index.B;
//        } else {
//            return Index.C;
//        }
        return selected;
    }

    @Override
    public double getRelative() {
        double pos = getEncoder();
        if (pos > 0) {
            pos -= Math.floor(pos / ticksPerRevolution) * ticksPerRevolution;
        }

        if (pos < 0) {
            pos -= Math.ceil(pos / ticksPerRevolution) * ticksPerRevolution;
        }
        return pos;
    }

    @Override
    public void update() {
            double currentPos = getEncoder();

            // Velocity (ticks per loop)
            double deltaMoved = currentPos - lastEncoderPos;
            velocity = deltaMoved;
            lastEncoderPos = currentPos;

            remainingDelta -= deltaMoved;

            // Position error
            positionError = remainingDelta;

            double error = remainingDelta;

            if (Math.abs(error) <= 5) {
                outputPower = Math.signum(error) * minPower;
                servo.setPower(outputPower);

                remainingDelta = 0;
                integral = 0;
                lastError = 0;
                kSActive = false;
                return;
            }

            integral += error;
            integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));

            double derivative = error - lastError;
            lastError = error;

            double power = kP * error + kI * integral + kD * derivative;

            // kS flag + application
            kSActive = Math.abs(error) > kSCutoff;
            if (kSActive) {
                power += Math.signum(error) * kS;
            }

            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

            double absError = Math.abs(error);
            if (absError < maxRampDistance) {
                double scale = absError / maxRampDistance;
                power = Math.signum(power) *
                        Math.max(minPower, Math.abs(power) * scale);
            }

            // Final output power (store what actually gets sent)
            outputPower = -power;
            servo.setPower(outputPower);
    }

    @Override
    public double getPositionError() {return positionError;}
    @Override
    public double getOutputPower() {return outputPower;}
    @Override
    public double getVelocity() {return velocity;}
    @Override
    public int getKSFlag() {return kSActive ? 1 : 0;}




}
