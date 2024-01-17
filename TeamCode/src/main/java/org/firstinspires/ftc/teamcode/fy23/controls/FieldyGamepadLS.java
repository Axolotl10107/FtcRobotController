package org.firstinspires.ftc.teamcode.fy23.controls;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.FriendlyIMU;
import org.firstinspires.ftc.teamcode.fy23.robot.units.DTS;

public class FieldyGamepadLS extends NoDriveAxes {

    private Gamepad driver;
    private Gamepad manipulator;

    private FriendlyIMU imu;

    private ElapsedTime buttonTimer;
    private int buttonRest = 300;

    private Vector2d rrVector;

    public FieldyGamepadLS(Gamepad initgamepad1, Gamepad initgamepad2, FriendlyIMU argIMU) {
        super(initgamepad1, initgamepad2);
        driver = initgamepad1;
        manipulator = initgamepad2;
        buttonTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        imu = argIMU;
    }

    public DTS dts() {
        rrVector = new Vector2d(GamepadInputs.leftStickYLinear(driver, 1), GamepadInputs.leftStickXLinear(driver, 1)).rotated(Math.toRadians(imu.yaw()));
//        return new DTS(
//                GamepadInputs.rightTriggerLinear(driver, 1) - GamepadInputs.leftTriggerLinear(driver, 1),
//                -GamepadInputs.leftStickXLinear(driver, 1),
//                GamepadInputs.rightStickXLinear(driver, 1)
//        );
        return new DTS(
                rrVector.component1(),
                GamepadInputs.leftTriggerLinear(driver, 1) - GamepadInputs.rightTriggerLinear(driver, 1),
                // Positive turn is counterclockwise!
                rrVector.component2()
        );
    }

    @Override
    public double forwardMovement() {
        return dts().drive;
    }

    @Override
    public double rotateMovement() {
        return dts().turn;
    }

    @Override
    public double strafeMovement() {
        return dts().strafe;
    }
}
