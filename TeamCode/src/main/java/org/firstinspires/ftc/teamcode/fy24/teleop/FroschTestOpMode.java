package org.firstinspires.ftc.teamcode.fy24.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy23.FieldyTeleOpScheme23;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy23.TeleOpScheme23;
import org.firstinspires.ftc.teamcode.fy23.gamepad2.teleop.fy23.TeleOpState23;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot24;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

@TeleOp
public class FroschTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot24 robot = new Robot24(RobotRoundhouse.getParamsAuto(hardwareMap), hardwareMap);
        TeleOpState23 controlsState;
        TeleOpScheme23 controlsScheme = new FieldyTeleOpScheme23(gamepad1, gamepad2, robot.imu);

        IMUCorrector.Parameters params = new IMUCorrector.Parameters(robot.imu, new TunablePID(robot.extendedParameters.hdgCorrectionPIDConsts));
        IMUCorrector imuCorrector = new IMUCorrector(params);

        waitForStart();

        while(opModeIsActive()) {
            robot.update();
            controlsState = controlsScheme.getState();
            DTS normalizedDTS = controlsState.getDts().normalize();
            robot.drive.applyDTS(imuCorrector.correctDTS(normalizedDTS));
            robot.drive.applyDTS(normalizedDTS);

            if (controlsState.isSquareUp()) {
                imuCorrector.squareUp();
            }

//            robot.arm.setPivotPower(controlsState.getArmMovement());
//            robot.arm.setElevatorPower(controlsState.getElevatorMovement());

            if (gamepad2.right_trigger > 0) {
                robot.arm.setElevatorPower(gamepad2.right_trigger*10);
            } else if (gamepad2.left_trigger > 0) {
                robot.arm.setElevatorPower(-gamepad2.left_trigger*10);
            } else {
                robot.arm.setElevatorPower(0);
            }

            robot.arm.setPivotPower(gamepad2.right_stick_y*10);


            robot.claw.setState(controlsState.getClawState());

            telemetry.addData("drive", normalizedDTS.drive);
            telemetry.addData("turn", normalizedDTS.turn);
            telemetry.addData("strafe", normalizedDTS.strafe);
            telemetry.addData("heading", robot.imu.yaw());
            telemetry.addData("heading error", imuCorrector.getHeadingError());
            telemetry.addData("haveHitTarget?", imuCorrector.hasHitTarget());
            telemetry.addData("turning?", imuCorrector.isTurning());
            telemetry.addLine("----------------------");
            telemetry.addData("arm position", robot.arm.getPivotPosition());
            telemetry.addData("elevator position", robot.arm.getElevatorPosition());
            telemetry.addData("arm velocity", robot.arm.getPivotVelocity());
            telemetry.addData("elevator velocity", robot.arm.getElevatorVelocity());
            telemetry.addData("arm power", robot.arm.getPivotPower());
            telemetry.addData("elevator power", robot.arm.getElevatorPower());

            telemetry.addData("triggers", -gamepad2.left_trigger + gamepad2.right_trigger);
            telemetry.addData("right stick", gamepad2.right_stick_y);

            telemetry.update();
        }
    }
}
