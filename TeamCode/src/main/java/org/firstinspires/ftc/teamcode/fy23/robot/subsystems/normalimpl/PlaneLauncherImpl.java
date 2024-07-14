package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.robotcore.hardware.Servo;

/** A normal implementation of {@link org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher}. */
public class PlaneLauncherImpl implements org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher {

    private Servo servo;
    private boolean launchComplete;

    public PlaneLauncherImpl(Parameters parameters) {
        servo = parameters.planeServo;
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
