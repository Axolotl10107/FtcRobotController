package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher;

/** A normal implementation of {@link PlaneLauncher}. */
public class PlaneLauncherImpl implements PlaneLauncher {

    private Servo servo;
    private boolean launchComplete;

    public PlaneLauncherImpl(Parameters parameters, HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, parameters.planeServoName);
    }

    @Override
    public void launch() {
        servo.setPosition(1);
    }

    @Override
    public void update() {
        // handle auto retract
        if (servo.getPosition() > 0.99) {
            launchComplete = true;
        }
        if (launchComplete) {
            servo.setPosition(0);
            launchComplete = false;
        }
    }

}
