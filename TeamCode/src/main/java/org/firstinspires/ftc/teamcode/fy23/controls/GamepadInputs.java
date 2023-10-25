package org.firstinspires.ftc.teamcode.fy23.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadInputs {
    static double rightStickYLinear(Gamepad gamepad, double scaling) {
        return gamepad.right_stick_y * scaling;
    }

    static double rightStickXLinear(Gamepad gamepad, double scaling) {
        return gamepad.right_stick_x * scaling;
    }

    static double leftStickXLinear(Gamepad gamepad, double scaling) {
        return gamepad.left_stick_x * scaling;
    }
//
//    double rightStickYExponential(int gamepad, double scaling) {
//        if (gamepad == 2) {
//            return -Math.pow(gamepad2.right_stick_y, scaling);
//        } else {
//            return -Math.pow(gamepad1.right_stick_y, scaling);
//        }
//    }
}
