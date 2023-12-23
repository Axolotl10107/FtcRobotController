//MediumAutomaton 2023
package org.firstinspires.ftc.teamcode.old_meddev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="New Combiner Test", group="Off-Season")
public class NewCombinerTest extends LinearOpMode {
    //Initialization
    FTC22Robot robot;
    private Controller controller;

    private double combinedLeftFront;
    private double combinedRightFront;
    private double combinedLeftBack;
    private double combinedRightBack;

    private double maxDrivePower = 0.5;
    private double turnStrafePower = 0.3;
    private double trigPoint = 0.3;
    private double maxElevPower = 0.5;

    private double drive;
    private double turn;
    private double strafe;

    private ElapsedTime ddeb = new ElapsedTime();//TODO: Handle toggle timers in the Controller

    @Override
    public void runOpMode() {
        robot = new FTC22Robot(hardwareMap);
        controller = new Controller(gamepad1, gamepad2);
//        controller.pollEverythingOpmode();
        telemetry.addData("Status", "[PowerNap] Initialized and waiting");
        waitForStart();
        telemetry.addData("Status", "[PowerNap] Running");
        while (opModeIsActive()) {
            //Refresh controller fields
            controller.pollEverythingOpmode();

            //Check for button presses and handle them
            if (controller.driveMaxPowerUp && ddeb.milliseconds() > 300) {
                maxDrivePower += 0.1;
                ddeb.reset();
            } else if (controller.driveMaxPowerDown && ddeb.milliseconds() > 300) {
                maxDrivePower -= 0.1;
                ddeb.reset();
            } else if (controller.turnStrafePowerUp && ddeb.milliseconds() > 300) {
                turnStrafePower += 0.1;
                ddeb.reset();
            } else if (controller.turnStrafePowerDown && ddeb.milliseconds() > 300) {
                turnStrafePower -= 0.1;
                ddeb.reset();
            } else if (controller.driver.start && ddeb.milliseconds() > 300) {
                trigPoint += 0.1;
                ddeb.reset();
            } else if (controller.driver.back && ddeb.milliseconds() > 300) {
                trigPoint -= 0.1;
                ddeb.reset();
            }

//            combinedLeftFront = controller.analogForward + Range.clip(controller.turnAxis, -1, 0) + Range.clip(controller.strafeAxis, -1, 0) - controller.analogReverse;
//            combinedRightFront = controller.analogForward - Range.clip(controller.turnAxis, 0, 1) + Range.clip(controller.strafeAxis, 0, 1) - controller.analogReverse;
//            combinedLeftBack = controller.analogForward + Range.clip(controller.turnAxis, 0, 1) + Range.clip(controller.strafeAxis, -1, 0) - controller.analogReverse;
//            combinedRightBack = controller.analogForward - Range.clip(controller.turnAxis, -1, 0) + Range.clip(controller.strafeAxis, -0, 1) - controller.analogReverse;

            drive = (controller.analogForward - controller.analogReverse)*maxDrivePower;
            turn = controller.turnAxis*turnStrafePower;
            strafe = controller.strafeAxis*turnStrafePower;

            if (drive > trigPoint) {
                telemetry.addData("Status", "Fast Combiner");
//                 combinedLeftFront = Range.clip(Range.clip(controller.analogForward - controller.analogReverse, -maxDrivePower, maxDrivePower) + Range.clip(controller.turnAxis*turnStrafePower, -turnStrafePower, 0), -maxDrivePower, maxDrivePower);//Adding a negative is the same as subtracting!
//                  combinedLeftBack = Range.clip(Range.clip(controller.analogForward - controller.analogReverse, -maxDrivePower, maxDrivePower) + Range.clip(controller.turnAxis*turnStrafePower, -turnStrafePower, 0), -maxDrivePower, maxDrivePower);//TODO: Don't need the overarching clip!
//                combinedRightFront = Range.clip(Range.clip(controller.analogForward - controller.analogReverse, -maxDrivePower, maxDrivePower) - Range.clip(controller.turnAxis*turnStrafePower, 0, turnStrafePower), -maxDrivePower, maxDrivePower);
//                 combinedRightBack = Range.clip(Range.clip(controller.analogForward - controller.analogReverse, -maxDrivePower, maxDrivePower) - Range.clip(controller.turnAxis*turnStrafePower, 0, turnStrafePower), -maxDrivePower, maxDrivePower);

//                 combinedLeftFront = Range.clip((controller.analogForward - controller.analogReverse)*maxDrivePower, -maxDrivePower, maxDrivePower) + Range.clip(controller.turnAxis*turnStrafePower, -turnStrafePower, 0);//Adding a negative is the same as subtracting!
//                  combinedLeftBack = Range.clip((controller.analogForward - controller.analogReverse)*maxDrivePower, -maxDrivePower, maxDrivePower) + Range.clip(controller.turnAxis*turnStrafePower, -turnStrafePower, 0);//TODO: Don't need the overarching clip!
//                combinedRightFront = Range.clip((controller.analogForward - controller.analogReverse)*maxDrivePower, -maxDrivePower, maxDrivePower) - Range.clip(controller.turnAxis*turnStrafePower, 0, turnStrafePower);
//                 combinedRightBack = Range.clip((controller.analogForward - controller.analogReverse)*maxDrivePower, -maxDrivePower, maxDrivePower) - Range.clip(controller.turnAxis*turnStrafePower, 0, turnStrafePower);

//                 combinedLeftFront = Range.clip(drive, -maxDrivePower, maxDrivePower) + Range.clip(turn, -turnStrafePower, 0);//Adding a negative is the same as subtracting!
//                  combinedLeftBack = Range.clip(drive, -maxDrivePower, maxDrivePower) + Range.clip(turn, -turnStrafePower, 0);//TODO: Don't need the overarching clip!
//                combinedRightFront = Range.clip(drive, -maxDrivePower, maxDrivePower) - Range.clip(turn, 0, turnStrafePower);
//                 combinedRightBack = Range.clip(drive, -maxDrivePower, maxDrivePower) - Range.clip(turn, 0, turnStrafePower);

                 combinedLeftFront = Range.clip(drive, -maxDrivePower, maxDrivePower) + Range.clip(turn, -turnStrafePower, 0) + Range.clip(strafe, -turnStrafePower, 0);//Adding a negative is the same as subtracting!
                  combinedLeftBack = Range.clip(drive, -maxDrivePower, maxDrivePower) + Range.clip(turn, -turnStrafePower, 0) - Range.clip(strafe, 0, turnStrafePower);
                combinedRightFront = Range.clip(drive, -maxDrivePower, maxDrivePower) - Range.clip(turn, 0, turnStrafePower) - Range.clip(strafe, 0, turnStrafePower);
                 combinedRightBack = Range.clip(drive, -maxDrivePower, maxDrivePower) - Range.clip(turn, 0, turnStrafePower) + Range.clip(strafe, -turnStrafePower, 0);
            } else if (-drive > trigPoint) {
                 combinedLeftFront = Range.clip(drive, -maxDrivePower, maxDrivePower) + Range.clip(turn, 0, turnStrafePower) + Range.clip(strafe, 0, turnStrafePower);//Adding a negative is the same as subtracting!
                  combinedLeftBack = Range.clip(drive, -maxDrivePower, maxDrivePower) + Range.clip(turn, 0, turnStrafePower)- Range.clip(strafe, -turnStrafePower, 0);
                combinedRightFront = Range.clip(drive, -maxDrivePower, maxDrivePower) - Range.clip(turn, -turnStrafePower, 0) - Range.clip(strafe, -turnStrafePower, 0);
                 combinedRightBack = Range.clip(drive, -maxDrivePower, maxDrivePower) - Range.clip(turn, -turnStrafePower, 0) + Range.clip(strafe, 0, turnStrafePower);
            } else {
                telemetry.addData("Status", "Slow Combiner");
//                combinedLeftFront = Range.clip(Range.clip(drive, -maxDrivePower, maxDrivePower) + Range.clip(turn, -turnStrafePower, turnStrafePower) + Range.clip(strafe, -turnStrafePower, turnStrafePower), -trigPoint, trigPoint);
//                combinedLeftBack = Range.clip(Range.clip(drive, -maxDrivePower, maxDrivePower) + Range.clip(turn, -turnStrafePower, turnStrafePower) - Range.clip(strafe, -turnStrafePower, turnStrafePower), -trigPoint, trigPoint);
//                combinedRightFront = Range.clip(Range.clip(drive, -maxDrivePower, maxDrivePower) - Range.clip(turn, -turnStrafePower, turnStrafePower) - Range.clip(strafe, -turnStrafePower, turnStrafePower), -trigPoint, trigPoint);
//                combinedRightBack = Range.clip(Range.clip(drive, -maxDrivePower, maxDrivePower) - Range.clip(turn, -turnStrafePower, turnStrafePower) + Range.clip(strafe, -turnStrafePower, turnStrafePower), -trigPoint, trigPoint);

                combinedLeftFront = Range.clip(drive, -trigPoint, trigPoint) + Range.clip(turn, -turnStrafePower, turnStrafePower) + Range.clip(strafe, -turnStrafePower, turnStrafePower);
                combinedLeftBack = Range.clip(drive, -trigPoint, trigPoint) + Range.clip(turn, -turnStrafePower, turnStrafePower) - Range.clip(strafe, -turnStrafePower, turnStrafePower);
                combinedRightFront = Range.clip(drive, -trigPoint, trigPoint) - Range.clip(turn, -turnStrafePower, turnStrafePower) - Range.clip(strafe, -turnStrafePower, turnStrafePower);
                combinedRightBack = Range.clip(drive, -trigPoint, trigPoint) - Range.clip(turn, -turnStrafePower, turnStrafePower) + Range.clip(strafe, -turnStrafePower, turnStrafePower);
            }
//            combinedLeftFront = controller.analogForward;
//            combinedLeftBack = gamepad1.right_trigger;
//            combinedRightBack = controller.analogForward;
//            combinedRightFront = gamepad1.right_trigger;

            //TODO: Maybe move to Robot class?
            robot.robotMap.leftFront.setPower(combinedLeftFront);
            robot.robotMap.rightFront.setPower(combinedRightFront);
            robot.robotMap.leftBack.setPower(combinedLeftBack);
            robot.robotMap.rightBack.setPower(combinedRightBack);


            //TODO: Elevator stuff


            telemetry.addData("maxDrivePower", maxDrivePower);
            telemetry.addData("turnStrafePower", turnStrafePower);
            telemetry.addData("trigPoint", trigPoint);

            telemetry.addData("trigValue", ((controller.analogForward - controller.analogReverse) * maxDrivePower));
            telemetry.addData("drive", drive);
            telemetry.addData("turn", turn);

            telemetry.addData("analogForward", controller.analogForward);
            telemetry.addData("turnAxis", controller.turnAxis);
            telemetry.addData("analogReverse", controller.analogReverse);

            telemetry.addData("combinedLeftFront", combinedLeftFront);
            telemetry.addData("combinedLeftBack", combinedLeftBack);
            telemetry.addData("combinedRightFront", combinedRightFront);
            telemetry.addData("combinedRightBack", combinedRightBack);

            telemetry.update();
        }
        telemetry.addData("Status", "[PowerNap] Stopped!");
    }
}
