//MediumAutomaton 2023, for FTC22Robot / PowerNap
package org.firstinspires.ftc.teamcode.old_meddev;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PowerNap", group="Off-Season")
public class PowerNap extends LinearOpMode {
    //Initialization
    FTC22Robot robot;
    private Controller controller;

    private double combinedLeftFront;
    private double combinedRightFront;
    private double combinedLeftBack;
    private double combinedRightBack;

    private double maxDrivePower = 0.7;
    private double turnStrafePower = 0.7;
    private double trigPoint = 0.3;
    private double maxElevPower = 0.5;

    private double drive;
    private double turn;
    private double strafe;
    private double elevPower;

    private ElapsedTime ddeb = new ElapsedTime();

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
            controller.pollSingle();
            //Check Elevator Limits
            robot.elevator.checkLimits();

            //Check for button presses and handle them
            if (controller.elevatorHigh) {
                robot.elevator.setTieredMode();
                robot.elevator.moveToHighTier();
            } else if (controller.elevatorMedium) {
                robot.elevator.setTieredMode();
                robot.elevator.moveToMediumTier();
            } else if (controller.elevatorLow) {
                robot.elevator.setTieredMode();
                robot.elevator.moveToLowTier();
            } else if (controller.elevatorStack) {
                robot.elevator.setTieredMode();
                robot.elevator.moveToStackTier();
            } else if (controller.elevatorGround) {
                robot.elevator.setTieredMode();
                robot.elevator.moveToGroundTier();
            } else if (controller.clawToggle) {
                robot.manipulator.toggleClaw();
            } else if (controller.armToggle) {
                robot.manipulator.toggleArm();
            } else if (controller.resetElevatorEncoder) {
                robot.elevator.setCurrentPositionAsZero();
            }

            drive = (controller.analogForward - controller.analogReverse)*maxDrivePower;
            turn = controller.turnAxis*turnStrafePower;
            strafe = controller.strafeAxis*turnStrafePower;

            if (drive > trigPoint) {
                telemetry.addData("Status", "Fast Combiner");
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
                combinedLeftFront = Range.clip(drive, -trigPoint, trigPoint) + Range.clip(turn, -turnStrafePower, turnStrafePower) + Range.clip(strafe, -turnStrafePower, turnStrafePower);
                combinedLeftBack = Range.clip(drive, -trigPoint, trigPoint) + Range.clip(turn, -turnStrafePower, turnStrafePower) - Range.clip(strafe, -turnStrafePower, turnStrafePower);
                combinedRightFront = Range.clip(drive, -trigPoint, trigPoint) - Range.clip(turn, -turnStrafePower, turnStrafePower) - Range.clip(strafe, -turnStrafePower, turnStrafePower);
                combinedRightBack = Range.clip(drive, -trigPoint, trigPoint) - Range.clip(turn, -turnStrafePower, turnStrafePower) + Range.clip(strafe, -turnStrafePower, turnStrafePower);
            }

            //TODO: Maybe move to Robot class?
            robot.robotMap.leftFront.setPower(combinedLeftFront);
            robot.robotMap.rightFront.setPower(combinedRightFront);
            robot.robotMap.leftBack.setPower(combinedLeftBack);
            robot.robotMap.rightBack.setPower(combinedRightBack);


//            elevPower = (controller.elevatorUp - controller.elevatorDown)*maxElevPower;

            if (controller.elevatorUpBool) {
                robot.elevator.setManualMode();
                elevPower = maxElevPower;
            } else if (controller.elevatorDownBool) {
                robot.elevator.setManualMode();
                elevPower = -maxElevPower;
            } else {
//                elevPower = 0;
                if (!robot.elevator.isBusy()) {
                    robot.elevator.holdPosition();
                }
            }

            if (controller.elevatorDownBool || controller.elevatorUpBool) {
                robot.elevator.setPower(Range.clip(elevPower, -maxElevPower/2, maxElevPower));
            }


            telemetry.addData("maxDrivePower", maxDrivePower);
            telemetry.addData("turnStrafePower", turnStrafePower);
            telemetry.addData("trigPoint", trigPoint);

            telemetry.addData("clawToggle", controller.clawToggle);
            telemetry.addData("armToggle", controller.armToggle);
            telemetry.addData("Manipulator toggle timer", robot.manipulator.toggleTimer);
            telemetry.addData("clawBusy?", robot.manipulator.clawServo.isBusy());
            telemetry.addData("armBusy?", robot.manipulator.armServo.isBusy());

            telemetry.addData("trigValue", ((controller.analogForward - controller.analogReverse) * maxDrivePower));
            telemetry.addData("drive", drive);
            telemetry.addData("turn", turn);
            telemetry.addData("elevatorHigh", controller.elevatorHigh);
            telemetry.addData("elevatorUpBool", controller.elevatorUpBool);
            telemetry.addData("Buffered elevPower", elevPower);
            telemetry.addData("Actual elevatorPower", robot.elevator.getPower());
            telemetry.addData("elevatorRunMode", robot.elevator.getMode());
//            telemetry.addData("elevatorLockoutTimer", robot.elevator.lockoutTimer);
            telemetry.addData("elevatorTargetPosition", robot.elevator.getTargetPosition());
            telemetry.addData("elevatorActualPosition", robot.elevator.getCurrentPosition());

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
