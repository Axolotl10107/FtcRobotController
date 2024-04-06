package org.firstinspires.ftc.teamcode.fy23.units;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TpSFinder extends OpMode {

    private DcMotorEx motor;
    private ElapsedTime stopwatch;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        stopwatch = new ElapsedTime();
    }

    @Override
    public void start() {
        stopwatch.reset();
        motor.setPower(0.5);
        while (stopwatch.milliseconds() < 5000) { /* be lazy */ } // let the motor spin up and settle in
        int pos1 = motor.getCurrentPosition();
        if (pos1 < 10) {
            telemetry.addLine("Position has not advanced - is the encoder connected properly?");
            telemetry.addData("Position at this error", pos1);
        }

        stopwatch.reset();
        while (stopwatch.milliseconds() < 1000) { /* be lazy */ }
        int pos2 = motor.getCurrentPosition();
        double actualTime1 = stopwatch.seconds();
        int dist1 = pos2 - pos1;
        int TpS1 = (int) (dist1 / actualTime1); // if it runs slightly over 1 second, take it back to 1 second

        stopwatch.reset();
        motor.setPower(1);
        while (stopwatch.milliseconds() < 5000) { /* be lazy */ }
        int pos3 = motor.getCurrentPosition();

        stopwatch.reset();
        while (stopwatch.milliseconds() < 1000) { /* be lazy */ }
        int pos4 = motor.getCurrentPosition();
        double actualTime2 = stopwatch.seconds();
        int dist2 = pos4 - pos3;
        int TpS2 = (int) (dist2 / actualTime2);

        motor.setPower(0);
        telemetry.addData("At half power:", TpS1);
        telemetry.addData("At full power:", TpS2);
    }

    @Override
    public void loop() {
        /* be lazy */
    }
}
