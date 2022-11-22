package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="NonBlockingDebounceTest", group="Test")
public class NonBlockingDebounceTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while(opModeIsActive()) {
            if (gamepad1.a) {
                if (timer.milliseconds() > 100) {
                    telemetry.addData("State", "In debounce delay");
                } else if (timer.milliseconds() > 1000) {
                    timer.reset();
                } else {
                    telemetry.addData("State", "Just pressed");
                }
            }
            if (gamepad1.b) {
                telemetry.addData("B State", "Look, you can still press me!");
            } else {
                telemetry.addData("B State", "Not Pressed");
            }
            telemetry.addData(" Millis", timer.milliseconds());
            telemetry.update();
        }
    }
}
