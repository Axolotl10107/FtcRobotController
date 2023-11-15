package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadDTS;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadInputs;
import org.firstinspires.ftc.teamcode.fy23.controls.GamepadLinear;


@TeleOp(name="Manipulator Opmode", group="Manipulator Opmode")
public class Manipulator_Code extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ArmMotor armPivot = null;
    private DcMotor tempMotor = null;
    private DcMotor armExtend = null;
    //motors for driving
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    double driveClip = 0.7;
    ElapsedTime driveClipDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        GamepadDTS Controls = new GamepadDTS(gamepad1, gamepad2);

        telemetry.addData("Status", "Ready for Initialisation");

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //code runs the arm up and down
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        armExtend.setDirection(DcMotor.Direction.REVERSE);

        tempMotor = hardwareMap.get(DcMotor.class, "armPivot");
        tempMotor.setDirection(DcMotor.Direction.REVERSE);
        armPivot = new ArmMotor(tempMotor);

        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        int elevatorLowerLimit = armExtend.getCurrentPosition();
        int elevatorUpperLimit = elevatorLowerLimit + 2500;

        int armDefaultPosition = armPivot.getCurrentPosition();
        armPivot.setTargetPosition(armDefaultPosition);

        double servoDefaultPosition = .585;
        clawServo.setPosition(servoDefaultPosition);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
    // controls the arm
            armPivot.runToPosition();
            if (Controls.armMovement() != 0) {
                armDefaultPosition += Controls.armMovement()*3;
                armPivot.setTargetPosition(armDefaultPosition);
            }

            if (gamepad1.start && driveClipDeb.milliseconds() > 300) {
                driveClip += 0.1;
                driveClipDeb.reset();
            }
            if (gamepad1.back && driveClipDeb.milliseconds() > 300) {
                driveClip -= 0.1;
                driveClipDeb.reset();
            }
            telemetry.addData("Max Drive Power", driveClip);

            /*
            Consider this - might not be what you want but it's a potential option:
            if (Controls.armMovement() != 0) {
                armPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPivot.setPower(Controls.armMovement());
            } else {
                armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            */

            if (Controls.armForward() > 0) {
                armPivot.setTargetPosition(armDefaultPosition);
            } else if (Controls.armBackward() > 0) {
                armPivot.setTargetPosition(armDefaultPosition + 1750);
            }

    // controls the elevator
            if (armExtend.getCurrentPosition() < elevatorLowerLimit) {
                armExtend.setPower(Range.clip(Controls.elevatorMovement(), 0,1));
            } else if (armExtend.getCurrentPosition() > elevatorUpperLimit) {
                armExtend.setPower(Range.clip(Controls.elevatorMovement(), -1, 0));
            } else {
                armExtend.setPower(Controls.elevatorMovement());

            }

    // controls the claw
            if (Controls.clawOpen() > 0) {
                clawServo.setPosition(servoDefaultPosition - .1); //Opens claw
            } else if (Controls.clawClose() > 0) {
                clawServo.setPosition(servoDefaultPosition); //Closes claw
            }
    // controls the wheels
            leftFront.setPower(Range.clip(Controls.forwardMovement() + Controls.strafeMovement() + Controls.rotateMovement(), -driveClip, driveClip));
            rightFront.setPower(Range.clip(Controls.forwardMovement() - Controls.strafeMovement() - Controls.rotateMovement(), -driveClip, driveClip));
            leftBack.setPower(Range.clip(Controls.forwardMovement() - Controls.strafeMovement() + Controls.rotateMovement(), -driveClip, driveClip));
            rightBack.setPower(Range.clip(Controls.forwardMovement() + Controls.strafeMovement() - Controls.rotateMovement(), -driveClip, driveClip));

            telemetry.update();
        }
    }
}
