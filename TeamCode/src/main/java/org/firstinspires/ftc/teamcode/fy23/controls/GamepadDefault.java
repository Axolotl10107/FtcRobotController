package org.firstinspires.ftc.teamcode.fy23.controls;

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
        double net = moveForward() - moveBackward();
        return net;
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
        double net = rotateClockwise() - rotateAnticlockwise(); //check to make sure this is actually right
        return net;
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
        double net = armForward() - armBackward();
        return net;
    }

    @Override
    public double elevatorMovement() {
        double net = elevatorUp() - elevatorDown();
        return net;
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
        double net = clawOpen() - clawClose();
        return net;
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
