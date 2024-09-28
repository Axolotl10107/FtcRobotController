package org.firstinspires.ftc.teamcode.fy23.autoSwitch;

import org.firstinspires.ftc.teamcode.fy23.*;
import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;

public class AutoSequence {
    private Robot robot;
    private String name; // unique identifier for the sequence. two sequences with the same name cannot be added to the same sequence switcher
    private TrajectorySequence sequence;

    public AutoSequence(String name, TrajectorySequence sequence) {
        this.name = name;
        this.sequence = sequence;
    }

    public String getName() {
        return name;
    }

    public TrajectorySequence getTrajectorySequence() {
        return sequence;
    }

    
}
