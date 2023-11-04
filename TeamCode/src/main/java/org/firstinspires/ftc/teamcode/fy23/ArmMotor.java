package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmMotor extends DcMotorWrapper{
    public ArmMotor(DcMotor m) {
        super(m);
    }
    @Override
    public void setTargetPosition(int i) {
        froschTargetPosition(i);
    }
    public void froschTargetPosition(int target) {
        int error = target - getCurrentPosition();
        double motorPower = Math.pow(error, 2) / 1000;
        setPower(motorPower);
    }
}
