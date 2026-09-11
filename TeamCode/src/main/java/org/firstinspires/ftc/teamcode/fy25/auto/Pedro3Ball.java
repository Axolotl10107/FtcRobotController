package org.firstinspires.ftc.teamcode.fy25.auto;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.fy25.ctlpad.IndyStarterBotScheme25;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Autonomous(name = "Pedro3Ball")
public class Pedro3Ball extends OpMode {
    private Robot25 robot;
    private Follower follower;
    public static Pose startingPose = new Pose(56, 8, Math.toRadians(90));
    private PathChain scoringPath;
    private PathChain intake1;
    private PathChain Path1;
    private TelemetryManager telemetryM;
    boolean read;
    int motif;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        scoringPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(60, 15))))
                .setLinearHeadingInterpolation(Math.toRadians(follower.getHeading()), Math.toRadians(110))
                .build();
        intake1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(40, 35))))
                .setLinearHeadingInterpolation(Math.toRadians(follower.getHeading()), Math.toRadians(180))
                .build();
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),

                                new Pose(60.000, 20.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))

                .build();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
//        robot.update();
        follower.update();
//        telemetryM.update();

        follower.followPath(Path1);
    }
}
