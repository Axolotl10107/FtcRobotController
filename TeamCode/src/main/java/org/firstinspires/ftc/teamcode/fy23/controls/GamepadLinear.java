package org.firstinspires.ftc.teamcode.fy23.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadLinear extends GamepadDefault {
    public GamepadLinear(Gamepad initgamepad1, Gamepad initgamepad2) {
        super(initgamepad1, initgamepad2);
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
}
