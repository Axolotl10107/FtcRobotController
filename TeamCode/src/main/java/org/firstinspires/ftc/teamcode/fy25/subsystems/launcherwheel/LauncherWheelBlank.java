package org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel;

/** A blank implementation of {@link LauncherWheel} that does nothing.
 * Never returns null (returns blank objects instead where applicable), so hopefully no null pointers. */
public class LauncherWheelBlank implements LauncherWheel {

    @Override
    public void spinUp() {

    }

    @Override
    public void spinDown() {

    }

    @Override
    public State getState() {
        return State.STOPPED;
    }

    @Override
    public void setLaunchRPM(double velocity) {

    }

//    @Override
//    public void fixLaunchSpin(double distance) {}
//
//    @Override
//    public void revertLaunchSpin() {
//
//    }

    @Override
    public void allowEntry() {}

    @Override
    public double getCurrentRPM() {
        return 0;
    }

    @Override
    public double getLaunchVelTargetRPM() {
        return 0;
    }

    @Override
    public void update() {

    }
}
