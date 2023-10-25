package org.firstinspires.ftc.teamcode.fy23.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadLinear implements GamepadInterface {
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    public GamepadLinear(Gamepad gamepad1, Gamepad gamepad2) {

    }
    @Override
    public double forwardMovement() {
        return GamepadInputs.rightStickYLinear(gamepad1, 1);
    }

    @Override
    public double strafeMovement() {
        return 0;
    }

    @Override
    public double rotateMovement() {
        return 0;
    }

    @Override
    public double armMovement() {
        return 0;
    }

    @Override
    public double elevatorMovement() {
        return 0;
    }

    @Override
    public double clawOpen() {
        return 0;
    }

    @Override
    public double clawClose() {
        return 0;
    }
}
