package org.firstinspires.ftc.teamcode.fy24.auto;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

public class AprilTagUtils {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // Constructor to initialize AprilTagProcessor and VisionPortal
    public AprilTagUtils(HardwareMap hardwareMap) {
        initAprilTagProcessor(hardwareMap);
    }

    private void initAprilTagProcessor(HardwareMap hardwareMap) {
        // Create the AprilTag processor with default settings
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(912, 912,630.884, 414.924) // Adjust these parameters as needed
                .build();

        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  // Set the camera (you can switch this to your desired camera)
                .addProcessor(aprilTagProcessor);  // Add the AprilTag processor

        visionPortal = builder.build();
    }

    // Method to get the distance from the detected AprilTag
    public double getDistanceY() {
        double distance = -1;
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        while (currentDetections == null || currentDetections.isEmpty()) {
            currentDetections = aprilTagProcessor.getDetections();
        }
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                distance = detection.ftcPose.y;  // Return the y-coordinate as the distance
            }
        }

        return distance;
    }

    public double getDistanceX() {
        double distance = -1;
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        while (currentDetections == null || currentDetections.isEmpty()) {
            currentDetections = aprilTagProcessor.getDetections();
        }
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                distance = detection.ftcPose.x;  // Return the y-coordinate as the distance
            }
        }

        return distance;
    }

    // Method to stop the VisionPortal (for cleaning up)
    public void stopVision() {
        visionPortal.close();
    }
}
