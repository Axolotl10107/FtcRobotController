package org.firstinspires.ftc.teamcode.fy24.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.fy23.controls.GamepadDefault;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadInputs;

public class GamepadDTS extends GamepadDefault {
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public GamepadDTS(Gamepad initgamepad1, Gamepad initgamepad2) {
        super(initgamepad1, initgamepad2);
        gamepad1 = initgamepad1;
        gamepad2 = initgamepad2;
    }

    @Override
    public double forwardMovement() {
        return GamepadInputs.rightTriggerLinear(gamepad1, 1) - GamepadInputs.leftTriggerLinear(gamepad1, 1);
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
        return GamepadInputs.rightTriggerLinear(gamepad2, 1);
    }

    @Override
    public double armBackward() {
        return GamepadInputs.leftTriggerLinear(gamepad2, 1);
    }

    public double armPivot() {return -GamepadInputs.leftStickYExponential(gamepad2, 1);}

    public double intakeServoIn() {
        return GamepadInputs.buttonA(gamepad2);
    }

    public double intakeServoOut() {
        return GamepadInputs.buttonB(gamepad2);
    }

    public double clawServoIn() {return GamepadInputs.buttonX(gamepad2);}

    public double clawServoOut() {return GamepadInputs.buttonY(gamepad2);}

    public double clawPivotUp() {return GamepadInputs.buttonDpadRight(gamepad2);}

    public double clawPivotDown() {return GamepadInputs.buttonDpadLeft(gamepad2);}

    public double brake() {return GamepadInputs.buttonX(gamepad1);}

    public double resetArmVars() {return GamepadInputs.rightBumper(gamepad2) * GamepadInputs.leftBumper(gamepad2);}

    public double rotateCamera() {return GamepadInputs.rightStickXLinear(gamepad2, 1);}

    @Override
    public double driveSpeedUp() {return GamepadInputs.buttonDpadRight(gamepad1);}

    @Override
    public double driveSpeedDown() {return GamepadInputs.buttonDpadLeft(gamepad1);}
}
