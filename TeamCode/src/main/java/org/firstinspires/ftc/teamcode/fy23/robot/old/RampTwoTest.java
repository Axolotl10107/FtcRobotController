package org.firstinspires.ftc.teamcode.fy23.robot.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.fy23.robot.Robot24;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;

@Disabled
@TeleOp
public class RampTwoTest extends OpMode {
    // TODO: I changed the wheelDiameter in the Robots to meters.

    Robot24 robot;
    RampTwo ramper;

    double currentPos;
    double suggestion;
    double toApply;

    boolean flag = false;

    double ticksToCM(double ticks) {
        return (ticks / robot.TPR) * robot.wheelCircumference;
    }

    double cmToTicks(double cm) {
        return (cm * robot.TPR) / robot.wheelCircumference;
    }

    private int getAvgEncoderPos() {
        return (
                robot.drive.getLeftFrontMotor().getCurrentPosition() +
                robot.drive.getRightFrontMotor().getCurrentPosition() +
                robot.drive.getLeftBackMotor().getCurrentPosition() +
                robot.drive.getRightBackMotor().getCurrentPosition()
        ) / 4;
    }

    @Override
    public void init() {
        robot = new Robot24(RobotRoundhouse.getRobotBParams(hardwareMap), hardwareMap);
        robot.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // This will be applied to all of the drive motors.

        ramper = new RampTwo(5, 10, 1, 2, 30, 0);
        // Accelerate by 5 cm/sec²
        // up to 10 cm/sec²
        // Accelerate by at most 1 cm/sec² each loop (if a loop takes way too long, don't jolt too much)
        // Don't go slower than 0.5 cm/sec. (otherwise we'll never reach the target, just get infinitely close to it)
        // Travel forward 30 cm
        // We are starting at 0cm (we have not made any previous progress towards our target)
    }

    @Override
    public void loop() {
        currentPos = getAvgEncoderPos(); // how far we've traveled in encoder ticks
        // above function not in interface
        currentPos = ticksToCM(currentPos); // convert it to centimeters so it's easier for us to think about

        if (flag == false) {
            suggestion = ramper.getSuggestionAtPos(currentPos); // What velocity should we set right now, according to our speed ramping?
        } else {
            suggestion = ramper.getSuggestionAtPos(-currentPos + 30); // tell it that it's going forward
        }
        toApply = cmToTicks(suggestion); // convert back to encoder ticks for the motors

        if (suggestion != 0 && flag == false) { // if it's forwards time
            robot.drive.getLeftFrontMotor().setVelocity(toApply);
            robot.drive.getRightFrontMotor().setVelocity(toApply);
            robot.drive.getLeftBackMotor().setVelocity(toApply);
            robot.drive.getRightBackMotor().setVelocity(toApply);
        } else if (suggestion != 0 && flag == true) { // if it's backwards time
            robot.drive.getLeftFrontMotor().setVelocity(-toApply);
            robot.drive.getRightFrontMotor().setVelocity(-toApply);
            robot.drive.getLeftBackMotor().setVelocity(-toApply);
            robot.drive.getRightBackMotor().setVelocity(-toApply);
        } else if (suggestion == 0 && flag == false) { // if it's time to switch
            if (!flag) {
                flag = true;
                ramper = new RampTwo(5, 10, 1, 2, 30, 0);
                // Start over!
            }
        } else { // if it's time to be all done
            requestOpModeStop();
        }

        telemetry.addData("currentPos", currentPos);
        telemetry.addData("suggestion", suggestion);
        telemetry.addData("toApply", toApply);
        telemetry.addData("flag", flag);
        telemetry.addLine("----------------------------");
        telemetry.addData("remainingDistance", ramper.remainingDistance);
        telemetry.addData("timeToDecel", ramper.timeToDecel);
        telemetry.addData("accelToStop", ramper.accelToStop);
    }
}
