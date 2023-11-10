package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmMotor extends DcMotorWrapper{
    private int targetPosition = 0;
    private double lastError;

    public ArmMotor(DcMotor m) {
        super(m);
    }

    @Override
    public void setTargetPosition(int i) {
        targetPosition = i;
    }

    @Override
    public int getTargetPosition() {
        return targetPosition;
    }

    public void runToPosition() {
        runToPosition(getTargetPosition());
    }
    public void runToPosition(int target) {
        double error = target - getCurrentPosition();
        lastError = error;
        if (lastError - error > 1) {
            error = -error;
        }
        double motorPower = (error * Math.abs(error)) / 1000;
        setPower(motorPower);
    }
}
