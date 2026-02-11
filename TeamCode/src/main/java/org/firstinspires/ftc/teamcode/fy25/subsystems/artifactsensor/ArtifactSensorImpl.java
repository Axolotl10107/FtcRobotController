package org.firstinspires.ftc.teamcode.fy25.subsystems.artifactsensor;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.ArrayList;
import java.util.List;

public class ArtifactSensorImpl implements ArtifactSensor {
    ColorSensor colorSensor;
    float greenHueMin;
    float greenHueMax;
    float purpleHueMin;
    float purpleHueMax;

    List<Artifact> holding = new ArrayList<>();

    public ArtifactSensorImpl(Parameters parameters) {
        colorSensor = parameters.colorSensor;
        float greenHueMin = parameters.greenHueMin;
        float greenHueMax = parameters.greenHueMax;
        float purpleHueMin  = parameters.purpleHueMin;
        float purpleHueMax = parameters.purpleHueMax;

        holding.add(Artifact.NONE);
        holding.add(Artifact.NONE);
        holding.add(Artifact.NONE);
    }

    @Override
    public Artifact readArtifact() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        boolean isGreen =
                hue >= 85 && hue <= 180;

        boolean isPurple =
                hue >= 210 && hue <= 400;

        if (val <= 0.1 || val >= 1 || (sat <= 0.5 && !isPurple)) {
            return Artifact.NONE;
        }

        if (isGreen) {
            return Artifact.GREEN;
        }
        if (isPurple) {
            return Artifact.PURPLE;
        }

        return Artifact.NONE;
    }

    @Override
    public float[] getHsv() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);

        return hsv;
    }

    @Override
    public void setHolding(int index, Artifact color) {
        holding.set(index, color);
    }

    @Override
    public List<Artifact> getHolding() {
        return holding;
    }
}
