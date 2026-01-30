package org.firstinspires.ftc.teamcode.fy25.subsystems.artifactsensor;

import android.graphics.Color;
import android.util.Pair;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class ArtifactSensorImpl implements ArtifactSensor {
    ColorSensor colorSensor;
    float greenHueMin;
    float greenHueMax;
    float purpleHueMin;
    float purpleHueMax;

    public ArtifactSensorImpl(Parameters parameters) {
        colorSensor = parameters.colorSensor;
        float greenHueMin = parameters.greenHueMin;
        float greenHueMax = parameters.greenHueMax;
        float purpleHueMin  = parameters.purpleHueMin;
        float purpleHueMax = parameters.purpleHueMax;
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

        if (sat < 0.20f) {
            return Artifact.NONE;
        }

        boolean isGreen =
                hue >= greenHueMin && hue <= greenHueMax;

        boolean isPurple =
                hue >= purpleHueMin && hue <= purpleHueMax;

        if (isGreen && !isPurple) return Artifact.GREEN;
        if (isPurple && !isGreen) return Artifact.PURPLE;

        return Artifact.NONE;
    }
}
