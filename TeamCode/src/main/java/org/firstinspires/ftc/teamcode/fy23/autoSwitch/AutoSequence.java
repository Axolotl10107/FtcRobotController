import org.firstinspires.ftc.teamcode.fy23;

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
