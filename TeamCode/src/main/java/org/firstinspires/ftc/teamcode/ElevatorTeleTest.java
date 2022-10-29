//ALoTO 2022-23
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Elevator Tele Test", group="Linear Opmode")

public class ElevatorTeleTest extends LinearOpMode {
    private DcMotor elevatorDrive;
    private DigitalChannel upperLimit;
    private DigitalChannel lowerLimit;
    private double maxPower = 0.5;//Change this to adjust the max motor power, or use the D-Pad to adjust it at runtime.

    @Override
    public void runOpMode() {
        elevatorDrive = hardwareMap.get(DcMotor.class, "ElevatorDrive");
        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        upperLimit = hardwareMap.get(DigitalChannel.class, "UpperLimit");
        upperLimit.setMode(DigitalChannel.Mode.INPUT);
        lowerLimit.setMode(DigitalChannel.Mode.OUTPUT);
        lowerLimit = hardwareMap.get(DigitalChannel.class, "UpperLimit");

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double up = gamepad1.right_trigger;
            double down = gamepad1.left_trigger;
            double upPower = Range.clip(up, 0, maxPower);
            double downPower = Range.clip(down, -maxPower, 0);
            telemetry.addData("        Up Trigger:", up);
            telemetry.addData("      Down Trigger:", down);
            telemetry.addData("         Max Power:", maxPower);
            telemetry.addData("  Clipped Up Power:", upPower);
            telemetry.addData("Clipped Down Power:", downPower);
            if (up > 0 && upperLimit.getState()) {//Limit switches are normally closed.
                elevatorDrive.setPower(upPower);
            } else if (down < 0 && lowerLimit.getState()) {
                elevatorDrive.setPower(downPower);
            } else if (gamepad1.dpad_up) {
                maxPower += 0.1;
            } else if (gamepad1.dpad_down) {
                maxPower -= 0.1;
            } else if (gamepad1.left_bumper) {
                double startTime = getRuntime();
                elevatorDrive.setPower(-maxPower);
                while (lowerLimit.getState()) {
                    telemetry.addData("Running for (seconds):", getRuntime()-startTime);
                }
                elevatorDrive.setPower(0);
            } else if (gamepad1.right_bumper) {
                double startTime = getRuntime();
                elevatorDrive.setPower(maxPower);
                while (upperLimit.getState()) {
                    telemetry.addData("Running for (seconds):", getRuntime()-startTime);
                }
                elevatorDrive.setPower(0);
            } else {
                elevatorDrive.setPower(0);
            }
            telemetry.update();
        }
    }
}
