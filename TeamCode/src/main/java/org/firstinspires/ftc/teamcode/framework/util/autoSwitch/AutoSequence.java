package org.firstinspires.ftc.teamcode.framework.util.autoSwitch;

import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class AutoSequence {
    private Robot25 robot;
    private final String name; // unique identifier for the sequence. two sequences with the same name cannot be added to the same sequence switcher
    private final TrajectorySequence sequence;

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
