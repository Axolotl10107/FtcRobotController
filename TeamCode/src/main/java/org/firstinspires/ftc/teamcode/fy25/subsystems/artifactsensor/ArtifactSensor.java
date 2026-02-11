package org.firstinspires.ftc.teamcode.fy25.subsystems.artifactsensor;

import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.List;

public interface ArtifactSensor {
    float[] getHsv();

    void setHolding(int index, Artifact color);

    List<Artifact> getHolding();

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
