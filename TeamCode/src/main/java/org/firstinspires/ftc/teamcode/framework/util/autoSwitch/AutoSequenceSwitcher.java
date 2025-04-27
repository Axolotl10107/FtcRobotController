package org.firstinspires.ftc.teamcode.framework.util.autoSwitch;

import java.util.HashMap;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class AutoSequenceSwitcher {
    private final HashMap<String, AutoSequence> sequences = new HashMap<>();
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
        if (sequences.size() == 1){
            selectFirst();
        }
    }

    public void addSequence(String name, TrajectorySequence sequence) {
        addSequence(new AutoSequence(name, sequence));
    }

    public AutoSequence getAutoSequence(String name) {
        return sequences.get(name);
    }

    public AutoSequence getAutoSequence(int number) {
        return sequences.get(getNames()[forceInBounds(number)]);
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

    public boolean hasName(String name) {
        String[] names = getNames();
        for (String s : names) {
            if (s.equals(name)) {
                return true;
            }
        }
        return false;
    }


    public AutoSequence selectName(String name) {
        if (!hasName(name)) {
            return null;
        }
        currentSequence = getAutoSequence(name);
        return currentSequence;
    }

    public AutoSequence selectNumber(int number) {
        if (!hasName(getNames()[number])) {
            return null;
        }
        currentSequence = getAutoSequence(getNames()[forceInBounds(number)]);
        return currentSequence;
    }

    public AutoSequence selectFirst() {
        if (sequences.size() == 0) {
            return null;
        }
        currentSequence = getAutoSequence(getNames()[0]);
        return currentSequence;
    }

    public AutoSequence getSelected() {
        return currentSequence;
    }

    public int getSequenceNumber(String name) {
        String[] names = getNames();
        for (int i = 0; i < names.length; i++) {
            if (names[i].equals(name)) {
                return i;
            }
        }
        return -1;
    }

    public int currentSequenceNumber() {
        String name = currentSequence.getName();
        return getSequenceNumber(name);
    }

    public AutoSequence selectNext() {
        int i = currentSequenceNumber();
        currentSequence = getAutoSequence(i+1);
        return currentSequence;
    }

    public AutoSequence selectPrevious() {
        int i = currentSequenceNumber();
        currentSequence = getAutoSequence(i-1);
        return currentSequence;
    }

    private int forceInBounds(int i) {
        if (i < 0) {
            i = getNames().length - 1;
        } else if (i >= getNames().length) {
            i = 0;
        }
        return i;
    }
}
