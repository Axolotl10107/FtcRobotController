package org.firstinspires.ftc.teamcode.fy24.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

@Deprecated
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
        return moveForward() - moveBackward();
    }

    @Override
    public double moveForward() {
        return 0;
    }

    @Override
    public double moveBackward() {
        return 0;
    }

    @Override
    public double strafeMovement() {
        double net = strafeRight() - strafeLeft(); //check to make sure this is actually right
        return 0;
    }

    @Override
    public double strafeLeft() {
        return 0;
    }

    @Override
    public double strafeRight() {
        return 0;
    }

    @Override
    public double rotateMovement() {
        return rotateClockwise() - rotateAnticlockwise();
    }

    @Override
    public double rotateClockwise() {
        return 0;
    }

    @Override
    public double rotateAnticlockwise() {
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
        return armForward() - armBackward();
    }

    @Override
    public double elevatorMovement() {
        return elevatorUp() - elevatorDown();
    }

    @Override
    public double elevatorUp() {
        return 0;
    }

    @Override
    public double elevatorDown() {
        return 0;
    }

    @Override
    public double clawMovement() {
        return clawOpen() - clawClose();
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

    @Override
    public double driveSpeedUp() {
        return 0;
    }

    @Override
    public double driveSpeedDown() {
        return 0;
    }
}
