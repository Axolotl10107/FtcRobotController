package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="NonBlockingDebounceTest", group="Test")
public class NonBlockingDebounceTest extends LinearOpMode {

    //About Debouncing
    /* When a button is pressed, the contacts inside have a tendency to bounce. Literally. It's too
     * fast for you to notice, but the computer sees it and thinks you're pressing it hundreds of
     * times a second. To solve this and only register one press, we add a small delay - 1 second in
     * this case, but normally it'd be quite a bit less - after first contact, and by then you'll
     * have stopped pressing the button.
     * This demonstration does it without using the sleep() function, using an ElapsedTime background
     * chronometer instead, so that other code can run during the debounce delay. This would make
     * button toggles feasible during TeleOp on the field! */

    // X button will toggle this to demonstrate async debounced toggles.
    private boolean xflag = false;
    // Y button will use this to show why non-debounced toggles don't work.
    private boolean yflag = false;

    @Override
    public void runOpMode() {
        // Create the timer for the A button
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        // Create the timer for the X button
        ElapsedTime xdeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        waitForStart();

        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                if (timer.milliseconds() > 100) {
//                    telemetry.addData("State", "In debounce delay");
//                } else if (timer.milliseconds() > 1000) {
//                    timer.reset();
//                } else {
//                    telemetry.addData("State", "Just pressed");
//                }

            if (gamepad1.a) {
                if (timer.milliseconds() > 10000) {// A 10-second delay here to demonstrate code
                    timer.reset();// Allow the button to be pressed again if it has been enough time
                    telemetry.addData("A State", "Did something!");
                } else {// Otherwise, it's too soon and could be a bounce.
                    telemetry.addData("A State", "Debounce still ongoing!");
                }
            } else {
                if (timer.milliseconds() < 10000) {
                    telemetry.addData("A State", "Debounce ongoing!");
                } else {
                    telemetry.addData("A State", "Not Pressed");
            }}

            if (gamepad1.b) {
                //Show how the B button will still respond while others are debouncing
                telemetry.addData("B State", "Look, you can still press me!");
            } else {
                telemetry.addData("B State", "Not Pressed");
            }

            if (gamepad1.y) {
                yflag = !yflag;//Setting the yflag boolean to not itself.
                //This is to show why non-debounced toggles won't work...
            }

            //X button demonstrates a much more practical toggle with a 1-second delay.
            if (gamepad1.x && xdeb.milliseconds() > 1000) {
                xdeb.reset();
                xflag = !xflag;
            }

            //Show the booleans toggled by the X and Y buttons, to demonstrate the debounce
            telemetry.addData(" X Flag", xflag);
            telemetry.addData(" Y Flag", yflag);
            telemetry.addData(" Millis", timer.milliseconds());
            telemetry.addData("Xmillis", xdeb.milliseconds());
            telemetry.update();
        }
    }
}

