//ALoTO 2022-23
//D-Pad max power adjustment moving by giant arbitrary amount (probably not debounced...)
//Motor not actually spinning...
//Put limit switch states in telemetry
//Down auto works, but not down trigger
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Motor Test", group="Linear Opmode")

public class MotorTest extends LinearOpMode {
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

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
//We still need to code this
        while (opModeIsActive()) {
            double up = gamepad1.right_trigger;
            double down = gamepad1.left_trigger;
            double upPower = Range.clip(up, 0, maxPower);
            double downPower = Range.clip(down, 0, maxPower);
            telemetry.addData("        Up Trigger:", up);
            telemetry.addData("      Down Trigger:", down);
            telemetry.addData("         Max Power:", maxPower);
            telemetry.addData("  Clipped Up Power:", upPower);
             telemetry.addData("Clipped Down Power:", downPower);

            telemetry.update();
        }
    }
}
