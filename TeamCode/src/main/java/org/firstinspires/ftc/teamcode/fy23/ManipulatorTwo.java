package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="ManipulatorTwo", group="TeleTest")
public class ManipulatorTwo extends OpMode {
    TouchSensor touch;

    @Override
    public void init() {
        touch = hardwareMap.get(TouchSensor.class, "pivotLowerLimit");
    }

    @Override
    public void loop() {
        telemetry.addData("Pressed", touch.isPressed());


    }
}
