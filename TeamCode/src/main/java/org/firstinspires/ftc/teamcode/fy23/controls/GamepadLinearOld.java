package org.firstinspires.ftc.teamcode.fy23.controls;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadLinearOld extends GamepadDefault {
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    public GamepadLinearOld(Gamepad initgamepad1, Gamepad initgamepad2) {
        gamepad1 = initgamepad1;
        gamepad2 = initgamepad2;
    }
    @Override
    public double forwardMovement() {
        return GamepadInputs.rightStickYLinear(gamepad1, 1);
    }

    @Override
    public double strafeMovement() {
        return GamepadInputs.rightStickXLinear(gamepad1, 1);
    }

    @Override
    public double rotateMovement() {
        return GamepadInputs.leftStickXLinear(gamepad1, 1);
    }

    @Override
    public double armForward() {
        return GamepadInputs.buttonB(gamepad2);
    }

    @Override
    public double armBackward() {
        return GamepadInputs.buttonY(gamepad2);
    }

    @Override
    public double armMovement() {
        double net = GamepadInputs.buttonDpadUp(gamepad2) - GamepadInputs.buttonDpadDown(gamepad2);
        return net;
    }


    public double armMediumMovement() {
        return 0;
    }


    public double armFastMovement() {
        return 0;
    }

    @Override
    public double elevatorMovement() {
        double net = GamepadInputs.rightTriggerLinear(gamepad2, 1) - GamepadInputs.leftTriggerLinear(gamepad2, 1);
        return net;
    }

    @Override
    public double clawOpen() {
        return GamepadInputs.buttonX(gamepad2);
    }

    @Override
    public double clawClose() {
        return GamepadInputs.buttonA(gamepad2);
    }

    @Override
    public double maxDrivePowerUp() {
        return 0;
    }

    @Override
    public double maxDrivePowerDown() {
        return 0;
    }

    @Override
    public double planeLaunch() {
        return 0;
    }


    public void runOpMode() throws InterruptedException {

    }
}
