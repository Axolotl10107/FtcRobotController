package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

public interface PlaneLauncher {

    class Parameters {
        public boolean present;
    }

    void launch();
    void handleAutoRetract();

}
