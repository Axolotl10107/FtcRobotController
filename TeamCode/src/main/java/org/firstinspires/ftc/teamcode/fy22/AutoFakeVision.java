/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//ALoTO 2022-23

package org.firstinspires.ftc.teamcode.fy22;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Disabled
@Autonomous(name="Auto Fake Vision", group="Auto Vision")

public class AutoFakeVision extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         LeftBack   = null;
    private DcMotor         RightBack  = null;
    private DcMotor         RightFront  = null;
    private DcMotor         LeftFront  = null;

    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     REVERSE_SPEED = -0.6;
    //static final double     STRAFE_FORWARD_SPEED = 0.6;
    //static final double     STRAFE_REVERSE_SPEED = 0.6;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        LeftBack  = hardwareMap.get(DcMotor.class, "LeftBack");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 0: Drive forward for 1 second, maybe land in the right parking zone?
        LeftBack.setPower(FORWARD_SPEED);
        RightBack.setPower(FORWARD_SPEED);
        RightFront.setPower(FORWARD_SPEED);
        LeftFront.setPower(FORWARD_SPEED);
        sleep(500);
        LeftBack.setPower(0);
        RightBack.setPower(0);
        RightFront.setPower(0);
        LeftFront.setPower(0);
        sleep(10000);


        // Step 1:   Strafe Left for 1 seconds
        LeftBack.setPower(FORWARD_SPEED);
        RightBack.setPower(REVERSE_SPEED);
        RightFront.setPower(FORWARD_SPEED);
        LeftFront.setPower(REVERSE_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.milliseconds() < 500)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
/*
        // Michael's Step: Drive Left for 1 second
        LeftBack.setPower(FORWARD_SPEED);
        RightBack.setPower(REVERSE_SPEED);
        RightFront.setPower(FORWARD_SPEED);
        LeftFront.setPower(REVERSE_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Go forward for 1 second
        LeftBack.setPower(FORWARD_SPEED);
        RightBack.setPower(FORWARD_SPEED);
        RightFront.setPower(FORWARD_SPEED);
        LeftFront.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //Go Backwards for 1 second
        LeftBack.setPower(REVERSE_SPEED);
        RightBack.setPower(REVERSE_SPEED);
        RightFront.setPower(REVERSE_SPEED);
        LeftFront.setPower(REVERSE_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds

        //spin (Not tested)
        LeftBack.setPower(FORWARD_SPEED);
        RightBack.setPower(REVERSE_SPEED);
        RightFront.setPower(REVERSE_SPEED);
        LeftFront.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

      //Spin other way (also not tested)
        LeftBack.setPower(REVERSE_SPEED);
        RightBack.setPower(FORWARD_SPEED);
        RightFront.setPower(FORWARD_SPEED);
        LeftFront.setPower(REVERSE_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            }
      */

        // Step 4:  Stop
        LeftBack.setPower(0);
        RightBack.setPower(0);
        RightFront.setPower(0);
        LeftFront.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
