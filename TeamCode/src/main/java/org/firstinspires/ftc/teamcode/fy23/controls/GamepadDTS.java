package org.firstinspires.ftc.teamcode.fy23.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadDTS extends GamepadDefault {
    private Gamepad driver;
    private Gamepad manipulator;
    public GamepadDTS(Gamepad initgamepad1, Gamepad initgamepad2) {
        super(initgamepad1, initgamepad2);
        driver = initgamepad1;
        manipulator = initgamepad2;
    }
    @Override
    public double forwardMovement() {
        return GamepadInputs.rightTriggerLinear(driver, 1) - GamepadInputs.leftTriggerLinear(driver, 1);
    }

    @Override
    public double strafeMovement() {
        return GamepadInputs.rightStickXLinear(driver, 1);
    }

    @Override
    public double rotateMovement() {
        return GamepadInputs.leftStickXLinear(driver, 1);
    }

    @Override
    public double armForward() {
        return GamepadInputs.buttonB(manipulator);
    }

    @Override
    public double armBackward() {
        return GamepadInputs.buttonY(manipulator);
    }

    @Override
    public double armMovement() {
        double net = GamepadInputs.buttonDpadUp(manipulator) - GamepadInputs.buttonDpadDown(manipulator);
        return net;
    }

    public double armMediumMovement() {
        double net = GamepadInputs.buttonDpadRight(manipulator) - GamepadInputs.buttonDpadLeft(manipulator);
        return net;
    }

    public double armFastMovement() {
        double y = manipulator.left_stick_y;
        if (y > 0) {
            return -GamepadInputs.leftStickYExponential(manipulator, 2);
            //negative because the Y-axis is inverted on the gamepad itself
        } else {
            return GamepadInputs.leftStickYExponential(manipulator, 2);
        }

    }

    @Override
    public double elevatorMovement() {
        double net = GamepadInputs.rightTriggerLinear(manipulator, 1) - GamepadInputs.leftTriggerLinear(manipulator, 1);
        return net;
    }

    @Override
    public double clawOpen() {
        return GamepadInputs.buttonX(manipulator);
    }

    @Override
    public double clawClose() {
        return GamepadInputs.buttonA(manipulator);
    }

    @Override
    public double maxDrivePowerUp() {
        return GamepadInputs.buttonStart(driver);
    }

    @Override
    public double maxDrivePowerDown() {
        return GamepadInputs.buttonBack(driver);
    }

    @Override
    public double planeLaunch() {
        return GamepadInputs.rightBumper(driver);
    }

}
