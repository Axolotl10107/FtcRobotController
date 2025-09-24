//package org.firstinspires.ftc.teamcode.fy24.auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.fy23.roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.fy23.robot.Robot24;
//import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
//import org.firstinspires.ftc.teamcode.fy23.units.DTS;
//
//@Autonomous(name = "AprilTagAuto")
//public class AprilTagAuto extends LinearOpMode {
//    Robot24 robot;
//    AprilTagUtils aprilTag;
//
//    private void brake() {
//        robot.drive.applyDTS(new DTS(0, 0, 0));
//    };
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new Robot24(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
//
//        aprilTag = new AprilTagUtils(hardwareMap);
//
//        Pose2d startPose = new Pose2d(0, 0, 0);
//        robot.drive.setPoseEstimate(startPose);
//
//        robot.drive.applyDTS(new DTS(0, 0, -0.5));
//
//        TrajectorySequence rotateToFirstBasket = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
//                .turn(45)
//                .build();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            while (aprilTag.getDistanceY() == -1) {
//                robot.drive.applyDTS(new DTS(0, 0, -0.5));
//            }
//
//            brake();
//
//            while (aprilTag.getDistanceY() > 5) {
//                robot.drive.applyDTS(new DTS(0.5, 0, 0));
//            }
//
//            brake();
//
//            while (aprilTag.getDistanceY() < 5) {
//                robot.drive.applyDTS(new DTS(-0.1, 0, 0));
//            }
//
//            brake();
//
//            while (aprilTag.getDistanceY() > 5) {
//                robot.drive.applyDTS(new DTS(0.05, 0, 0));
//            }
//
//            brake();
//
//            robot.drive.followTrajectorySequence(rotateToFirstBasket);
//
//            brake();
//        }
//    }
//}
