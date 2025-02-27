package org.firstinspires.ftc.teamcode.fy23.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadInputs {
// sticks linear
    public static double rightStickYLinear(Gamepad gamepad, double scaling) {
        return -gamepad.right_stick_y * scaling;
    }

    public static double rightStickXLinear(Gamepad gamepad, double scaling) {
        return gamepad.right_stick_x * scaling;
    }

    static double leftStickYLinear(Gamepad gamepad, double scaling) {
        return -gamepad.left_stick_y * scaling;
    }

    public static double leftStickXLinear(Gamepad gamepad, double scaling) {
        return gamepad.left_stick_x * scaling;
    }
// sticks exponential
    static double rightStickYExponential(Gamepad gamepad, double scaling) {   
        return Math.pow(gamepad.right_stick_y, scaling);
    }

    static double rightStickXExponential(Gamepad gamepad, double scaling) {   
        return Math.pow(gamepad.right_stick_x, scaling);
    }

    public static double leftStickYExponential(Gamepad gamepad, double scaling) {
        return Math.pow(gamepad.left_stick_y, scaling);
    }

    static double leftStickXExponential(Gamepad gamepad, double scaling) {   
        return Math.pow(gamepad.left_stick_x, scaling);
    }
// triggers linear
    public static double rightTriggerLinear(Gamepad gamepad, double scaling) {
        return gamepad.right_trigger * scaling;
    }

    public static double leftTriggerLinear(Gamepad gamepad, double scaling) {
        return gamepad.left_trigger * scaling;
    }
// triggers exponential
    static double rightTriggerExponential(Gamepad gamepad, double scaling) {
        return Math.pow(gamepad.right_trigger, scaling);
    }

    static double leftTriggerExponential(Gamepad gamepad, double scaling) {
        return Math.pow(gamepad.left_trigger, scaling);
    }
// buttons
    public static double buttonA(Gamepad gamepad){
        if (gamepad.a) {
            return 1;
        } else {
            return 0;
        }
    }

    public static double buttonB(Gamepad gamepad){
        if (gamepad.b) {
            return 1;
        } else {
            return 0;
        }
    }

    public static double buttonX(Gamepad gamepad){
        if (gamepad.x) {
            return 1;
        } else {
            return 0;
        }
    }

    public static double buttonY(Gamepad gamepad){
        if (gamepad.y) {
            return 1;
        } else {
            return 0;
        }
    }

    static double buttonStart(Gamepad gamepad){
        if (gamepad.start) {
            return 1;
        } else {
            return 0;
        }
    }

    static double buttonBack(Gamepad gamepad) {
        if (gamepad.back) {
            return 1;
        } else {
            return 0;
        }
    }

    public static double buttonDpadDown(Gamepad gamepad){
        if (gamepad.dpad_down) {
            return 1;
        } else {
            return 0;
        }
    }

    public static double buttonDpadUp(Gamepad gamepad){
        if (gamepad.dpad_up) {
            return 1;
        } else {
            return 0;
        }
    }

    public static double buttonDpadRight(Gamepad gamepad){
        if (gamepad.dpad_right) {
            return 1;
        } else {
            return 0;
        }
    }

    public static double buttonDpadLeft(Gamepad gamepad){
        if (gamepad.dpad_left) {
            return 1;
        } else {
            return 0;
        }
    }

    public static double rightBumper(Gamepad gamepad){
        if (gamepad.right_bumper) {
            return 1;
        } else {
            return 0;
        }
    }

    public static double leftBumper(Gamepad gamepad){
        if (gamepad.left_bumper) {
            return 1;
        } else {
            return 0;
        }
    }
}
