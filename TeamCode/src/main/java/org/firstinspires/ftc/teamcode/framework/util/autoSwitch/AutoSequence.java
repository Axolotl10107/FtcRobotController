package org.firstinspires.ftc.teamcode.framework.util.autoSwitch;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;

public class AutoSequence {
    private Robot24 robot;
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
