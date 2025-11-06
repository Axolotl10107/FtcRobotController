package org.firstinspires.ftc.teamcode.fy24.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.framework.util.autoSwitch.AutoSequenceSwitcher;
//import org.firstinspires.ftc.teamcode.fy23.controls.ctlpad.FieldyTeleOpScheme23;
import org.firstinspires.ftc.teamcode.fy24.robots.Robot24;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse24;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
@Autonomous(name="Switch Autonomous Modes")
public class AutonomousSequences extends LinearOpMode {

    Robot24 robot;
    Telemetry opModeTelemetry;
    String autoMode = "1.0";

    private DcMotorEx armLeftExtend = null;
    private DcMotorEx armRightExtend = null;
    private DcMotorEx armLeftPivot = null;
    private DcMotorEx armRightPivot = null;

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

        armLeftExtend = hardwareMap.get(DcMotorEx.class, "armLeftExtend");
        armRightExtend = hardwareMap.get(DcMotorEx.class, "armRightExtend");
        armLeftPivot = hardwareMap.get(DcMotorEx.class, "armLeftPivot");
        armRightPivot = hardwareMap.get(DcMotorEx.class, "armRightPivot");

        armLeftExtend.setDirection(DcMotorSimple.Direction.FORWARD);
        armRightExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        armLeftPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        armRightPivot.setDirection(DcMotorSimple.Direction.FORWARD);

        armLeftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRightExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double armExtendSpeed = 1110;
        double armPivotSpeed = 1115;

        final double ticksPerInch = 157.86;
        final double ticksPerDegree = 32.06;

        CRServo servoIntake;
        Servo servoClaw;
        servoIntake = hardwareMap.get(CRServo.class, "intakeServo");
        servoClaw = hardwareMap.get(Servo.class, "clawServo");

        initAprilTag();

        opModeTelemetry = telemetry;

        robot = new Robot24(RobotRoundhouse24.getRobotAParams(hardwareMap), hardwareMap);

//        FieldyTeleOpScheme23 controlScheme = new FieldyTeleOpScheme23(gamepad1, gamepad2, robot.imu);

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

// All previous sequences are tests. Below this comment are true sequences.

        switcher.addSequence("3.0", robot.drive.trajectorySequenceBuilder(startPose)
                .forward(15)
                .turn(Math.toRadians(-135))
                .addTemporalMarker(5, () -> {
                    while (true) {
                        if (((armRightPivot.getCurrentPosition() + armLeftExtend.getCurrentPosition()) / 2) / ticksPerDegree < 75) {
                            armLeftPivot.setVelocity(armPivotSpeed);
                            armRightPivot.setVelocity(armPivotSpeed);
                        } else {
                            armLeftPivot.setVelocity(0);
                            armRightPivot.setVelocity(0);
                        }

                        if (((armRightExtend.getCurrentPosition() + armLeftExtend.getCurrentPosition()) / 2) / ticksPerInch < 48) {
                            armLeftExtend.setVelocity(armExtendSpeed);
                            armRightExtend.setVelocity(armExtendSpeed);
                        } else {
                            armLeftExtend.setVelocity(0);
                            armRightExtend.setVelocity(0);
                        }

                        if (((armRightExtend.getCurrentPosition() + armLeftExtend.getCurrentPosition()) / 2) / ticksPerInch >= 48 && ((armRightPivot.getCurrentPosition() + armLeftExtend.getCurrentPosition()) / 2) / ticksPerDegree >= 75) {
                            break;
                        }
                    }
                    servoIntake.setPower(-1);
                })
                .addTemporalMarker(15, () -> {
                    while (true) {
                        if (((armRightPivot.getCurrentPosition() + armLeftExtend.getCurrentPosition()) / 2) / ticksPerDegree > 0) {
                            armLeftPivot.setVelocity(-armPivotSpeed);
                            armRightPivot.setVelocity(-armPivotSpeed);
                        } else {
                            armLeftPivot.setVelocity(0);
                            armRightPivot.setVelocity(0);
                        }

                        if (((armRightExtend.getCurrentPosition() + armLeftExtend.getCurrentPosition()) / 2) / ticksPerInch > 0) {
                            armLeftExtend.setVelocity(-armExtendSpeed);
                            armRightExtend.setVelocity(-armExtendSpeed);
                        } else {
                            armLeftExtend.setVelocity(0);
                            armRightExtend.setVelocity(0);
                        }

                        if (((armRightExtend.getCurrentPosition() + armLeftExtend.getCurrentPosition()) / 2) / ticksPerInch <= 0 && ((armRightPivot.getCurrentPosition() + armLeftExtend.getCurrentPosition()) / 2) / ticksPerDegree <= 0) {
                            break;
                        }
                    }
                    servoIntake.setPower(0);
                })
                .turn(Math.toRadians(45))
                .strafeLeft(96)
                .turn(Math.toRadians(-90))
                .strafeRight(12)
                .build()
        );

        switcher.selectName(autoMode);


        while (true) {
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
                case 34:
                    autoMode = "3.0";
                    break;
            }

            break;
        }   // end for() loop
    }
}
