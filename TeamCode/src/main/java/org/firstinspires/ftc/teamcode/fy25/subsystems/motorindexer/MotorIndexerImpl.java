package org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer;

import static org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer.MotorIndexerImpl.MotorIndexerTuning.MAX_POWER;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer.MotorIndexerImpl.MotorIndexerTuning.kD;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer.MotorIndexerImpl.MotorIndexerTuning.kI;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer.MotorIndexerImpl.MotorIndexerTuning.kP;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer.MotorIndexerImpl.MotorIndexerTuning.kS;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer.MotorIndexerImpl.MotorIndexerTuning.kSCutoff;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer.MotorIndexerImpl.MotorIndexerTuning.maxIntegral;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer.MotorIndexerImpl.MotorIndexerTuning.maxRampDistance;
import static org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer.MotorIndexerImpl.MotorIndexerTuning.minPower;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.Indexer;

public class MotorIndexerImpl implements MotorIndexer {

    private final DcMotor motor;
    private final DcMotorEx encoder;

    private final TouchSensor limitSwitch;

    private final double ticksPerRevolution = 8192;
    private final double ticksPerIndex = 2730;

    private final double intakeTicks = 1200;

    private double integral = 0;
    private double lastError = 0;
    private Indexer.Index selected = Indexer.Index.A;
    private double remainingDelta = 0;
    private double lastEncoderPos;
    private boolean prepping = false;

    private double positionError = 0;
    private double outputPower = 0;
    private double velocity = 0;
    private boolean kSActive = false;

    @Config
    public static class MotorIndexerTuning {
        public static double MAX_POWER = 1;
        public static double kD = 0.00007;
        public static double kI = 0.00005;
        public static double kP = 0.00006;
        public static double kS = 0.0006;
        public static double kSCutoff = 1600;
        public static double maxIntegral = 850;
        public static double maxRampDistance = 0;
        public static double minPower = 0;
    }


    public MotorIndexerImpl(MotorIndexer.Parameters parameters) {
        motor = parameters.indexerMotor;
        encoder = parameters.encoderMotor;
        limitSwitch = parameters.limitSwitch;

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        motor.setPower(1);
    }

    @Override
    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void goTo(Indexer.Index index) {
        int numIndexes = Indexer.Index.values().length;
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
        selected = Indexer.Index.values()[
                (selected.ordinal() + 1) % Indexer.Index.values().length
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

        double deltaMoved = currentPos - lastEncoderPos;
        velocity = deltaMoved;
        lastEncoderPos = currentPos;

        remainingDelta -= deltaMoved;

        positionError = remainingDelta;

        double error = remainingDelta;

        if (Math.abs(error) <= 5) {
            outputPower = Math.signum(error) * minPower;
            motor.setPower(outputPower);

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

        outputPower = -power;
        motor.setPower(outputPower);
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
