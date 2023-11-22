package org.firstinspires.ftc.teamcode.fy23.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadDefault implements GamepadInterface {
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public GamepadDefault(Gamepad initgamepad1, Gamepad initgamepad2) {
        gamepad1 = initgamepad1;
        gamepad2 = initgamepad2;
    }

    public GamepadDefault() {
    }

    @Override
    public double forwardMovement() {
        return 0;
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
    public double armForward() {
        return 0;
    }

    @Override
    public double armBackward() {
        return 0;
    }

    @Override
    public double armMovement() {
        return 0;
    }

    @Override
    public double armMediumMovement() {
        return 0;
    }

    @Override
    public double armFastMovement() {
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

    @Override
    public double planeLaunch() {
        return 0;
    }
}
