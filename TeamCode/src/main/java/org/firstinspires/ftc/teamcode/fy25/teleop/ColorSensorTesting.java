package org.firstinspires.ftc.teamcode.fy25.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.framework.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.framework.util.TelemetrySingleton;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.IndyStarterBotScheme25;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;

@TeleOp(name="color sensor")
public class ColorSensorTesting extends OpMode {
    Robot25 robot;

    IMUCorrector imuCorrector;

    IndyStarterBotScheme25 controlScheme;

    ColorSensor colorSensor;

    @Override
    public void init() {
        TelemetrySingleton.setInstance(telemetry);

        robot = new Robot25(RobotRoundhouse25.getRobotBParams(hardwareMap), hardwareMap);

        imuCorrector = new IMUCorrector( robot.extendedParameters.imuCorrectorParams );

        controlScheme = new IndyStarterBotScheme25( gamepad1, gamepad2 );

        colorSensor = hardwareMap .get(ColorSensor.class, "colorSensor");
    }

    @Override
    public void loop() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (blue > 100) {
            telemetry.addData("color", "indexer");
        } else if (red > 30 && green > 30 && (double) red / green > 0.7) {
            telemetry.addData("color", "purple");
        } else if (red > 30) {
            telemetry.addData("color", "green");
        } else {
            telemetry.addData("color", "none");
        }

        telemetry.addData("red", red);
        telemetry.addData("green", green);
        telemetry.addData("blue", blue);
    }
}
