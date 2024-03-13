package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

public interface FriendlyIMU {

    class Parameters {
        public boolean present;
    }

    double pitch();
    double roll();
    double yaw();

}
