package org.firstinspires.ftc.teamcode.fy23.robot.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.fy23.robot.RobotB;
import org.firstinspires.ftc.teamcode.fy23.robot.VirtualRobot;
import org.firstinspires.ftc.teamcode.fy23.robot.generators.RampTwo;

@TeleOp
public class RampTwoTest extends OpMode {

    RobotB robot;
    RampTwo ramper;

    double currentPos;
    double suggestion;
    double toApply;

    boolean flag = false;

    double ticksToCM(double ticks) {
        return (ticks / robot.TPR) * robot.wheelDiameter;
    }

    double cmToTicks(double cm) {
        return (cm * robot.TPR) / robot.wheelDiameter;
    }

    @Override
    public void init() {
        robot = new RobotB(hardwareMap);
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
        currentPos = robot.drive.getRobotBAvgEncoderPos(); // how far we've traveled in encoder ticks
        currentPos = ticksToCM(currentPos); // convert it to centimeters so it's easier for us to think about

        if (flag == false) {
            suggestion = ramper.getSuggestionAtPos(currentPos); // What velocity should we set right now, according to our speed ramping?
        } else {
            suggestion = ramper.getSuggestionAtPos(-currentPos + 30); // tell it that it's going forward
        }
        toApply = cmToTicks(suggestion); // convert back to encoder ticks for the motors

        if (suggestion != 0 && flag == false) { // if it's forwards time
            robot.drive.setVelocity(toApply);
        } else if (suggestion != 0 && flag == true) { // if it's backwards time
            robot.drive.setVelocity(-toApply);
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
