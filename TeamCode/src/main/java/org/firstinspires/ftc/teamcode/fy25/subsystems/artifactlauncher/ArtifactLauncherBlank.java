package org.firstinspires.ftc.teamcode.fy25.subsystems.artifactlauncher;

/** A blank implementation of {@link ArtifactLauncher} that does nothing.
 * Never returns null (returns blank objects instead where applicable), so hopefully no null pointers. */
public class ArtifactLauncherBlank implements ArtifactLauncher {

    @Override
    public void readyLauncher() {

    }

    @Override
    public State getState() {
        return State.STOPPED;
    }

    @Override
    public void setVelocity(int velocity) {

    }

    @Override
    public int getVelocity() {
        return 0;
    }

    @Override
    public void update() {

    }
}
