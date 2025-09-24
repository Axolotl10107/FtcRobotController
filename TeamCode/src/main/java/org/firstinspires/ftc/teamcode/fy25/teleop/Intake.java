package org.firstinspires.ftc.teamcode.fy25.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.IndyStarterBotScheme25;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.StarterBotState25;

@TeleOp(name="Intake")
public class Intake extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");

        StarterBotState25 state = new StarterBotState25();
        IndyStarterBotScheme25 controls = new IndyStarterBotScheme25(gamepad1, gamepad2);

        waitForStart();
        while(opModeIsActive()) {
            if (controls.getState().getIntakeState() == RotaryIntake.State.RUNIN) {
                intakeMotor.setPower(1);
            } else if (controls.getState().getIntakeState() == RotaryIntake.State.RUNOUT) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }
        }
    }
}
