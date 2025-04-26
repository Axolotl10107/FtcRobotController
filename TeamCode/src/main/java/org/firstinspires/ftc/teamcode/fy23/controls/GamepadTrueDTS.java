package org.firstinspires.ftc.teamcode.fy23.controls;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.framework.units.DTS;
@Deprecated
public class GamepadTrueDTS extends GamepadDefault {

    private Gamepad driver;
    private Gamepad manipulator;

    private ElapsedTime buttonTimer;
    private int buttonRest = 300;

    public GamepadTrueDTS(Gamepad initgamepad1, Gamepad initgamepad2) {
        super(initgamepad1, initgamepad2);
        driver = initgamepad1;
        manipulator = initgamepad2;
        buttonTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public DTS dts() {
        return new DTS(
                GamepadInputs.rightTriggerLinear(driver, 1) - GamepadInputs.leftTriggerLinear(driver, 1),
                -GamepadInputs.leftStickXLinear(driver, 1),
                GamepadInputs.rightStickXLinear(driver, 1)
        );
    }

    public boolean pUp() {
        if (GamepadInputs.buttonDpadUp(driver) == 1 && buttonTimer.milliseconds() > buttonRest) {
            buttonTimer.reset();
            return true;
        } else {
            return false;
        }
    }

    public boolean pDown() {
        if (GamepadInputs.buttonDpadDown(driver) == 1 && buttonTimer.milliseconds() > buttonRest) {
            buttonTimer.reset();
            return true;
        } else {
            return false;
        }
    }

    public boolean imUp() {
        if (GamepadInputs.buttonY(driver) == 1 && buttonTimer.milliseconds() > buttonRest) {
            buttonTimer.reset();
            return true;
        } else {
            return false;
        }
    }

    public boolean imDown() {
        if (GamepadInputs.buttonA(driver) == 1 && buttonTimer.milliseconds() > buttonRest) {
            buttonTimer.reset();
            return true;
        } else {
            return false;
        }
    }

    public boolean dmUp() {
        if (GamepadInputs.buttonB(driver) == 1 && buttonTimer.milliseconds() > buttonRest) {
            buttonTimer.reset();
            return true;
        } else {
            return false;
        }
    }

    public boolean dmDown() {
        if (GamepadInputs.buttonX(driver) == 1 && buttonTimer.milliseconds() > buttonRest) {
            buttonTimer.reset();
            return true;
        } else {
            return false;
        }
    }

    public boolean changeUp() {
        if (GamepadInputs.buttonStart(driver) == 1 && buttonTimer.milliseconds() > buttonRest) {
            buttonTimer.reset();
            return true;
        } else {
            return false;
        }
    }

    public boolean changeDown() {
        if (GamepadInputs.buttonBack(driver) == 1 && buttonTimer.milliseconds() > buttonRest) {
            buttonTimer.reset();
            return true;
        } else {
            return false;
        }
    }

    public boolean hdgUp() {
        if (GamepadInputs.buttonDpadRight(driver) == 1 && buttonTimer.milliseconds() > buttonRest) {
            return true;
        } else {
            return false;
        }
    }

    public boolean hdgDown() {
        if (GamepadInputs.buttonDpadLeft(driver) == 1 && buttonTimer.milliseconds() > buttonRest) {
            return true;
        } else {
            return false;
        }
    }

    public boolean save() {
        if (GamepadInputs.rightBumper(driver) == 1 && buttonTimer.milliseconds() > buttonRest) {
            return true;
        } else {
            return false;
        }
    }
}
