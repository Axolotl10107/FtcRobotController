package org.firstinspires.ftc.teamcode.fy23.teletest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp()
public class PowerTeleTest extends OpMode {

    DcMotor motor;
    double setPower;
    ElapsedTime buttonTimer = new ElapsedTime();

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "armPivot");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && buttonTimer.milliseconds() > 200) {
            setPower += 0.01;
            buttonTimer.reset();
        }
        if (gamepad1.dpad_down && buttonTimer.milliseconds() > 200) {
            setPower -= 0.01;
            buttonTimer.reset();
        }
        motor.setPower(setPower);
        telemetry.addData("Your power", setPower);
        telemetry.addData("Motor's power", motor.getPower());
    }
}
