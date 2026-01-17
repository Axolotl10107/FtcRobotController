package org.firstinspires.ftc.teamcode.fy25.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.fy25.ctlpad.IndyStarterBotScheme25;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.StarterBotState25;

@TeleOp(name="NewBotTesting")
public class NewBotTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        StarterBotState25 state = new StarterBotState25();
        IndyStarterBotScheme25 controls = new IndyStarterBotScheme25(gamepad1, gamepad2);
    }
}
