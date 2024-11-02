package org.firstinspires.ftc.teamcode.fy24.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.fy23.autoSwitch.AutoSequenceSwitcher;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.FieldyTeleOpScheme;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="Switch Autonomous Modes")
public class FakeVision extends LinearOpMode {

    Robot robot;
    Telemetry opModeTelemetry;
    String autoMode = "1.0";

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    public AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        opModeTelemetry = telemetry;

        robot = new Robot(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);

        FieldyTeleOpScheme controlScheme = new FieldyTeleOpScheme(gamepad1, gamepad2, robot.imu);

        AutoSequenceSwitcher switcher = new AutoSequenceSwitcher();

        Pose2d startPose = new Pose2d(0, 0, 0);

        switcher.addSequence("1.0", robot.drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(10, 10), 0)
                .build()
        );

        switcher.addSequence("1.1", robot.drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-10, -10), 0)
                .build()
        );
        switcher.addSequence("1.2", robot.drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(10, -10), 0)
                .build()
        );

        switcher.selectName(autoMode);


        while(true) {
            if (gamepad1.a) {
                break;
            } else {
                processAprilTag();
                telemetry.update();
                sleep(10);
                switcher.selectName(autoMode);
            }

            telemetry.addData("current automode", switcher.getSelected().getName());
            telemetry.update();
        }

        robot.drive.followTrajectorySequence(switcher.getSelected().getTrajectorySequence());

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    public void processAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            switch (detection.id) {
                case 30:
                    autoMode = "1.0";
                    break;
                case 31:
                    autoMode = "1.1";
                    break;
                case 32:
                    autoMode = "1.2";
            }
            telemetry.addData("autoMode: ", autoMode);

            break;
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}
