package org.firstinspires.ftc.teamcode.fy25.subsystems.artifactsensor;

public class ArtifactSensorBlank implements ArtifactSensor {
    @Override
    public float[] getHsv() {
        return new float[0];
    }

    @Override
    public Artifact readArtifact() {
        return null;
    }
}
