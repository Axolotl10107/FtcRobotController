//ALoTO 2022-23
//Start + A sets controller to Gamepad 1
//Start + B sets controller to Gamepad 2
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Servo Tele Test", group="Linear Opmode")

public class ServoTeleTest extends LinearOpMode {
    private DcMotor elevatorDrive;
    private Servo servo1;
    private Servo servo2;
    private double maxPower = 0.5;

    @Override
    public void runOpMode() {
        elevatorDrive = hardwareMap.get(DcMotor.class, "Ellyvader");
        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
//We still need to code this
        while (opModeIsActive()) {
            double up = gamepad2.right_trigger;
            double down = gamepad2.left_trigger;
            double upPower = Range.clip(up, 0, maxPower);
            double downPower = Range.clip(down, 0, maxPower);
            telemetry.addData("         Up Trigger", up);
            telemetry.addData("       Down Trigger", down);
            telemetry.addData("          Max Power", maxPower);
            telemetry.addData("   Clipped Up Power", upPower);
            telemetry.addData(" Clipped Down Power", downPower);
            telemetry.addData("Servo Left Position", servo1.getPosition());
            telemetry.addData("Servo Right Position", servo2.getPosition());
            elevatorDrive.setPower(upPower - downPower);
            if (gamepad2.left_bumper) {
                servo1.setPosition(0);
            } else if (gamepad2.right_bumper) {
                servo1.setPosition(1);
            } else {
                servo1.setPosition((gamepad2.left_stick_x / 2) + 0.5);
                //+ 0.5 - stick goes -1 to 1, servo goes 0 to 1. This offsets the stick range.
                //Div. stick position by 2, so that, with the offset, 0 and 1 are at edges of stick
            }
            servo2.setPosition((gamepad2.right_stick_x / 2) + 0.5);
            telemetry.update();
        }
    }
}
