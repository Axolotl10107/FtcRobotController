package org.firstinspires.ftc.teamcode.fy23.subsystems.planelauncher;

import com.qualcomm.robotcore.hardware.Servo;

/** A normal implementation of {@link PlaneLauncher}.
 * Deprecated because no physical hardware for this exists right now (real-world testing is impossible). */
@Deprecated
public class PlaneLauncherImpl implements PlaneLauncher {

    private Servo servo;
    private boolean launchComplete = false;

    private double releasePosition;
    private double restPosition;

    public PlaneLauncherImpl(Parameters parameters) {
        servo = parameters.planeServo;
        releasePosition = parameters.releasePosition;
        restPosition = parameters.restPosition;
    }

    @Override
    public void launch() {
        servo.setPosition(releasePosition);
    }

    @Override
    /** Called by robot.update(). You do not need to call this method. */
    public void update() {
        // handle auto retract (in a way that only calls servo.setPosition() once, mostly as an example of tracking state)
        if (!launchComplete) {
            if (servo.getPosition() > (releasePosition - 0.01)) {
                launchComplete = true;
                servo.setPosition(restPosition);
            }
        }
    }

}
