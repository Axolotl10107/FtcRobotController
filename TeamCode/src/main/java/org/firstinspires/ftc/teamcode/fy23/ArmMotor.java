package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmMotor extends DcMotorWrapper{
    public ArmMotor(DcMotor m) {
        super(m);
    }
    public void runToPosition() {
        int error = getTargetPosition() - getCurrentPosition();
        double motorPower = (error * Math.abs(error)) / 1000;
        setPower(motorPower);
    }
    public void runToPosition(int target) {
        int error = target - getCurrentPosition();
        double motorPower = (error * Math.abs(error)) / 1000;
        setPower(motorPower);
    }
}
