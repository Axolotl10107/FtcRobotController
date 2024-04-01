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
    /** Called by robot.update(). You do not need to call this method. */
    public void update() {
        // handle auto retract (contrived to run servo.setPosition() exactly once and evaluate one, not two, conditions
        // on most loops)
        if (launchComplete) {
            servo.setPosition(0);
            launchComplete = false;
        } else if (servo.getPosition() > 0.99) {
            launchComplete = true;
        }
    }

}
