package org.firstinspires.ftc.teamcode.fy26.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.adapters.RangeLimitServo;
import org.firstinspires.ftc.teamcode.framework.ctlpad.primitives.buttons.ToggleButton;

@TeleOp(name = "Catapult Test", group = "FY26 TeleOp")
public class CatapultTest extends LinearOpMode {

    static final double BILLY_ENGAGED = 0.4;
    static final double BILLY_DISENGAGED = 0.6;
    static final double JANE_ENGAGED = 0.4;
    static final double JANE_DISENGAGED = 0.6;

    // The Release a
    private DcMotor gerogeTheCatapultMotor = null;
    private Servo billyTheServoThatReleasesTheDcMotor = null;
    private Servo janeTheSewvoThatWeleasesTheCatapult = null;

    private ToggleButton billyButton = null;
    private ToggleButton janeButton = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        gerogeTheCatapultMotor = hardwareMap.get(DcMotor.class, "geroge");
        billyTheServoThatReleasesTheDcMotor = hardwareMap.get(Servo.class, "billy");
        janeTheSewvoThatWeleasesTheCatapult = hardwareMap.get(Servo.class, "jane");

        // Wrap the servos to limit their range.
        billyTheServoThatReleasesTheDcMotor = new RangeLimitServo(billyTheServoThatReleasesTheDcMotor, BILLY_ENGAGED, BILLY_DISENGAGED);
        janeTheSewvoThatWeleasesTheCatapult = new RangeLimitServo(janeTheSewvoThatWeleasesTheCatapult, JANE_ENGAGED, JANE_DISENGAGED);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        gerogeTheCatapultMotor.setDirection(DcMotor.Direction.FORWARD);
        billyTheServoThatReleasesTheDcMotor.setDirection(Servo.Direction.FORWARD);
        janeTheSewvoThatWeleasesTheCatapult.setDirection(Servo.Direction.FORWARD);

        billyTheServoThatReleasesTheDcMotor.setPosition(BILLY_ENGAGED);
        janeTheSewvoThatWeleasesTheCatapult.setPosition(JANE_ENGAGED);

        billyButton = new ToggleButton(() -> gamepad1.a);
        janeButton = new ToggleButton(() -> gamepad1.right_trigger > 0.4);

        // Update telemetry at the end of initialization.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.b) {
                gerogeTheCatapultMotor.setPower(0.2);
            } else {
                gerogeTheCatapultMotor.setPower(0.0);
            }

            if (billyButton.isActive()) {
                billyTheServoThatReleasesTheDcMotor.setPosition(BILLY_DISENGAGED);
            } else {
                billyTheServoThatReleasesTheDcMotor.setPosition(BILLY_ENGAGED);
            }

            if (janeButton.isActive()) {
                janeTheSewvoThatWeleasesTheCatapult.setPosition(JANE_DISENGAGED);
            } else {
                janeTheSewvoThatWeleasesTheCatapult.setPosition(JANE_ENGAGED);
            }

            // Update telemetry at the end of each loop.
            telemetry.update();
        }
    }
}
