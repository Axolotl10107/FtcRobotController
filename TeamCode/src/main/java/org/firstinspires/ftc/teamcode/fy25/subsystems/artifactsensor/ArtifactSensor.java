package org.firstinspires.ftc.teamcode.fy25.subsystems.artifactsensor;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.ColorSensor;

public interface ArtifactSensor {
    enum Artifact {
        NONE,
        GREEN,
        PURPLE
    }

    class Parameters {
        public Parameters(boolean present) {
            this.present = present;
        }
        final public boolean present;
        public ColorSensor colorSensor;
        public float greenHueMin;
        public float greenHueMax;
        public float purpleHueMin;
        public float purpleHueMax;
    }
    public Artifact readArtifact();
}
