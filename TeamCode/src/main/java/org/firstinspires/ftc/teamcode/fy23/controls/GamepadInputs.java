package org.firstinspires.ftc.teamcode.fy23.controls;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadInputs {
    static double rightStickYLinear(Gamepad gamepad, double scaling) {
        return gamepad.right_stick_y * scaling;
    }

//    double rightStickXLinear(int gamepad, double scaling) {
//        if (gamepad == 2) {
//            return gamepad2.right_stick_x * scaling;
//        } else {
//            return gamepad1.right_stick_x * scaling;
//        }
//    }
//
//    double leftStickXLinear(int gamepad, double scaling) {
//        if (gamepad == 2) {
//            return gamepad2.left_stick_x * scaling;
//        } else {
//            return gamepad1.left_stick_x * scaling;
//        }
//    }
//
//    double rightStickYExponential(int gamepad, double scaling) {
//        if (gamepad == 2) {
//            return -Math.pow(gamepad2.right_stick_y, scaling);
//        } else {
//            return -Math.pow(gamepad1.right_stick_y, scaling);
//        }
//    }
}