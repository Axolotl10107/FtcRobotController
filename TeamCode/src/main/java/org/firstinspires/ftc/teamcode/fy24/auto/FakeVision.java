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

        robot.drive.setPoseEstimate(startPose);

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
        switcher.addSequence("2.0", robot.drive.trajectorySequenceBuilder(startPose)
                .forward(100)
                .strafeLeft(100)
                .back(100)
                .strafeRight(100)
                .waitSeconds(3)
                .lineToConstantHeading(new Vector2d(10, 100))
                .lineToConstantHeading(new Vector2d(-10, -100))
                .build()
        );

        switcher.selectName(autoMode);


        while(true) {
            if (gamepad1.a) {
                break;
            } else {
                processAprilTag();
                sleep(10);
                switcher.selectName(autoMode);
            }

            telemetry.addData("current automode", switcher.getSelected().getName());
            telemetry.update();
        }

        waitForStart();

        robot.drive.followTrajectorySequence(switcher.getSelected().getTrajectorySequence());

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }   // end method initAprilTag()

    public void processAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

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
                    break;
                case 33:
                    autoMode = "2.0";
                    break;
            }

            break;
        }   // end for() loop
    }
}
