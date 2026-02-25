package org.firstinspires.ftc.teamcode.fy25.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Deadwheel Raw Test", group = "Test")
public class DeadwheelLocalizer extends OpMode {
    Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {

        follower.update();

        Pose pose = follower.getPose();

        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (deg)",
                Math.toDegrees(pose.getHeading()));

        telemetry.update();
    }
}