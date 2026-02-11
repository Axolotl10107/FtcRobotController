package org.firstinspires.ftc.teamcode.fy25.subsystems.motifreader;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class MotifReaderImpl implements MotifReader {
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    public MotifReaderImpl(Parameters parameters) {
        visionPortal = parameters.visionPortal;
        aprilTag = parameters.aprilTag;
    }

    @Override
    public int getMotif() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 21) {
                return 21;
            } else if (detection.id == 22) {
                return 22;
            } else if (detection.id == 23) {
                return 23;
            }
        }
        return -1;
    }

    @Override
    public void closePortal() {
        visionPortal.close();
    }

    @Override
    public void update() {

    }
}
