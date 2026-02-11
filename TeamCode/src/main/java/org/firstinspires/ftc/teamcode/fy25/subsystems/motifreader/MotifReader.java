package org.firstinspires.ftc.teamcode.fy25.subsystems.motifreader;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public interface MotifReader {
    class Parameters {
        public Parameters(boolean present) {
            this.present = present;
        }
        public final boolean present;
        public VisionPortal visionPortal;
        public AprilTagProcessor aprilTag;
    }

    int getMotif();
    void closePortal();
    void update();
}
