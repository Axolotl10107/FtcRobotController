package org.firstinspires.ftc.teamcode.fy25.teleop;
//import static org.firstinspires.ftc.teamcode.fy25.teleop.PPTeleOp.OpModeAngleTuning.leftDown;
//import static org.firstinspires.ftc.teamcode.fy25.teleop.PPTeleOp.OpModeAngleTuning.leftUp;
//import static org.firstinspires.ftc.teamcode.fy25.teleop.PPTeleOp.OpModeAngleTuning.rightDown;
//import static org.firstinspires.ftc.teamcode.fy25.teleop.PPTeleOp.OpModeAngleTuning.rightUp;

import android.icu.util.RangeValueIterator;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoTypes;

import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.IndyStarterBotScheme25;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.StarterBotState25;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;
import org.firstinspires.ftc.teamcode.fy25.subsystems.artifactsensor.ArtifactSensor;
import org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.Indexer;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergateservo.LauncherGateServo;
import org.firstinspires.ftc.teamcode.fy25.subsystems.loader.Loader;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class PPTeleOp extends OpMode {
    private Robot25 robot;
    private IndyStarterBotScheme25 controlScheme;
    private Follower follower;
    public static Pose startingPose = new Pose(56, 8, Math.toRadians(90));
    private boolean automatedDrive;
    private Supplier<PathChain> scoringPath;
    private TelemetryManager telemetryM;
    boolean read;
    int motif;

    Servo servoRight;
    Servo servoLeft;

    @Override
    public void init() {
        robot = new Robot25(RobotRoundhouse25.getRobotBParams(hardwareMap), hardwareMap);
        controlScheme = new IndyStarterBotScheme25( gamepad1, gamepad2 );
        read = false;
        motif = -1;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        scoringPath = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(60, 15))))
                .setLinearHeadingInterpolation(Math.toRadians(follower.getHeading()), Math.toRadians(110))
                .build();

        servoRight = hardwareMap.get(Servo.class, "rampServoRight");
        servoLeft = hardwareMap.get(Servo.class, "rampServoLeft");
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        StarterBotState25 controlState = controlScheme.getState();
        robot.update();
        follower.update();
        telemetryM.update();
        telemetry.update();

        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // true for robot centric, false for field centric
            );
        }

        if (gamepad1.aWasPressed()) {
            follower.followPath(scoringPath.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

//        if (gamepad2.right_bumper) {
//            servoLeft.setPosition(leftUp);
//            servoRight.setPosition(rightUp);
//        } else if (gamepad2.left_bumper) {
//            servoLeft.setPosition(leftDown);
//            servoRight.setPosition(rightDown);
//        }

        if (controlState.getLauncherGateServoState() == LauncherGateServo.State.OPEN) {
            robot.launchGateServo.open();
        } else if (controlState.getLauncherGateServoState() == LauncherGateServo.State.CLOSED) {
            robot.launchGateServo.close();
        }

        if (controlState.getLoaderState() == Loader.State.LOAD) {
            robot.loader.load();
            if (controlState.isRunLaunchWheel()) {
                robot.artifactSensor.setHolding(robot.motorIndexer.getIndex().ordinal(), ArtifactSensor.Artifact.NONE);
            }
        } else {
            robot.loader.pass();
        }

        if (controlState.getIntakeState() == RotaryIntake.State.RUNIN) {
            robot.motorIntake.spinIn();
        } else {
            robot.motorIntake.stop();
        }

        ArtifactSensor.Artifact color = robot.artifactSensor.readArtifact();
//        if (controlState.getManualOverrideState() != 0) {
//            robot.indexer.manualOverride((int) controlState.getManualOverrideState());
//            robot.indexer.resetEncoder();
//            robot.motorIndexer.manualOverride((int) controlState.getManualOverrideState());
//            robot.motorIndexer.resetEncoder();
//        } else
        if (controlState.getIndexState() == Indexer.State.NEXT) {
            robot.indexer.next();
            robot.motorIndexer.next();
            controlState.setIndexState(Indexer.State.READY);
        } else if (controlState.getIndexState() == Indexer.State.PREP && color == ArtifactSensor.Artifact.NONE) {
            robot.indexer.prepIntake();
            robot.motorIndexer.prepIntake();
        } else if (controlState.getIndexState() == Indexer.State.PREP && color != ArtifactSensor.Artifact.NONE) {
            robot.indexer.intake();
            robot.motorIndexer.intake();
            controlState.setIndexState(Indexer.State.READY);
            robot.indexer.next();
            robot.motorIndexer.next();
            robot.artifactSensor.setHolding(robot.motorIndexer.getIndex().ordinal(), color);
        } else if (controlState.getIndexState() == Indexer.State.TO) {
            robot.indexer.goTo(robot.indexer.getIndex());
            robot.motorIndexer.goTo(robot.motorIndexer.getIndex());
        }

        if (motif == -1) {
            motif = robot.motifReader.getMotif();
        }

        if (controlState.getSort()) {
            switch (motif) {
                case 21:
                    if (robot.artifactSensor.getHolding().get(0) == ArtifactSensor.Artifact.GREEN) {
                        robot.indexer.goTo(Indexer.Index.A);
                        break;
                    }
                    if (robot.artifactSensor.getHolding().get(1) == ArtifactSensor.Artifact.GREEN) {
                        robot.indexer.goTo(Indexer.Index.B);
                        break;
                    }
                    if (robot.artifactSensor.getHolding().get(2) == ArtifactSensor.Artifact.GREEN) {
                        robot.indexer.goTo(Indexer.Index.C);
                        break;
                    }
                case 22:
                    if (robot.artifactSensor.getHolding().get(0) == ArtifactSensor.Artifact.GREEN) {
                        robot.indexer.goTo(Indexer.Index.C);
                        break;
                    }
                    if (robot.artifactSensor.getHolding().get(1) == ArtifactSensor.Artifact.GREEN) {
                        robot.indexer.goTo(Indexer.Index.A);
                        break;
                    }
                    if (robot.artifactSensor.getHolding().get(2) == ArtifactSensor.Artifact.GREEN) {
                        robot.indexer.goTo(Indexer.Index.B);
                        break;
                    }
                case 23:
                    if (robot.artifactSensor.getHolding().get(0) == ArtifactSensor.Artifact.GREEN) {
                        robot.indexer.goTo(Indexer.Index.B);
                        break;
                    }
                    if (robot.artifactSensor.getHolding().get(1) == ArtifactSensor.Artifact.GREEN) {
                        robot.indexer.goTo(Indexer.Index.C);
                        break;
                    }
                    if (robot.artifactSensor.getHolding().get(2) == ArtifactSensor.Artifact.GREEN) {
                        robot.indexer.goTo(Indexer.Index.A);
                        break;
                    }
            }
        }

        if (controlState.isRunLaunchWheel()) {
            robot.launchWheelSimple.spinUp(28000);
        } else {
            robot.launchWheelSimple.spinDown();
        }

        switch (controlState.getIntakeState()) {
            case RUNIN:
                robot.rotaryIntake.setState(RotaryIntake.State.RUNIN);
                read = false;
                break;
            case RUNOUT:
                robot.rotaryIntake.setState(RotaryIntake.State.RUNOUT);
                read = false;
                break;
            case STOPPED:
            default:
//                ArtifactSensor.Artifact color = robot.artifactSensor.readArtifact();
//                if (!read && color != ArtifactSensor.Artifact.NONE) {
//                    read = true;
//                } else if (!read) {
//                    robot.artifactSensor.setHolding(robot.indexer.getIndex().ordinal(), color);
//                }
                robot.rotaryIntake.setState(RotaryIntake.State.STOPPED);
                robot.indexer.intake();
                controlState.setIndexState(Indexer.State.READY);
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);

        telemetry.addData("rampPosL", servoLeft.getPosition());
        telemetry.addData("rampPosR", servoRight.getPosition());
    }
}