package org.firstinspires.ftc.teamcode.fy25.subsystems.artifactsensor;

import java.util.Collections;
import java.util.List;

public class ArtifactSensorBlank implements ArtifactSensor {
    @Override
    public float[] getHsv() {
        return new float[0];
    }

    @Override
    public void setHolding(int index, Artifact color) {

    }

    @Override
    public List<Artifact> getHolding() {
        return Collections.emptyList();
    }

    @Override
    public Artifact readArtifact() {
        return null;
    }
}
