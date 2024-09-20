import java.util.HashMap;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.fy23;

public class AutoSequenceSwitcher {
    private HashMap<String, AutoSequence> sequences = new HashMap<String, AutoSequence>();
    private AutoSequence currentSequence;

    public AutoSequenceSwitcher() {}

    public void addSequence(AutoSequence sequence) {
        String name = sequence.getName();
        for (String i : sequences.keySet()) {
            if (i.equals(name)) {
                return;
            }
        }
        sequences.put(name, sequence);
        return;
    }

    public void addSequence(String name, TrajectorySequence sequence) {
        addSequence(new AutoSequence(name, sequence));
        return;
    }

    public AutoSequence getAutoSequence(String name) {
        return sequences.get(name);
    }

    public TrajectorySequence getTrajectorySequence(String name) {
        return getAutoSequence(name).getTrajectorySequence();
    }

    public String[] getNames() {
        String[] names = new String[sequences.size()];
        int i = 0;
        for (String n : sequences.keySet()) {
            names[i] = n;
            i++;
        }
        return names;
    }
}
