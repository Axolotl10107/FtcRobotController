package org.firstinspires.ftc.teamcode.fy25.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.fy24.controls.GamepadDTS;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.IndyStarterBotScheme25;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.StarterBotState25;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;



@TeleOp(name="FY25 DraftTeleOp")
public class DraftTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //    private ArmMotor armPivot = null;
    //motors for driving
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    double driveClip = 0.7;
    ElapsedTime driveClipDeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    Telemetry.Log log = telemetry.log();

    @Override
    public void runOpMode() {
        try {
            realOpMode();
        } catch (Exception x) {
            log.add(x.getStackTrace().toString());
            throw x;
        }
    }
    public void realOpMode() {

        GamepadDTS controlsOld = new GamepadDTS(gamepad1, gamepad2);

        StarterBotState25 state = new StarterBotState25();
        IndyStarterBotScheme25 controls = new IndyStarterBotScheme25(gamepad1, gamepad2);

        telemetry.addData("Status", "Ready for Initialisation");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        runtime.reset();

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            if (gamepad1.start && driveClip < 1 && driveClipDeb.milliseconds() > 300) {
                driveClip += 0.1;
                driveClipDeb.reset();
            }
            if (gamepad1.back && driveClip > 0.25 && driveClipDeb.milliseconds() > 300) {
                driveClip -= 0.1;
                driveClipDeb.reset();
            }

            if (controls.getState().getIntakeState() == RotaryIntake.State.RUNIN) {
                intakeMotor.setPower(1);
            } else if (controls.getState().getIntakeState() == RotaryIntake.State.RUNOUT) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            // Change speed

            if (controlsOld.driveSpeedUp() != 0 && driveClip < 1 && driveClipDeb.milliseconds() > 300) {
                driveClip = 1;
                driveClipDeb.reset();
            } else if (controlsOld.driveSpeedDown() != 0 && driveClip > 0.25 && driveClipDeb.milliseconds() > 300) {
                driveClip = 0.3;
                driveClipDeb.reset();
            }

            // Brake

            if (controlsOld.brake() != 0) {
                double currentVelocity = leftFront.getVelocity();
                while (leftFront.getVelocity() > 0 && currentVelocity > 0) {
                    leftFront.setVelocity(-leftFront.getVelocity());
                    rightFront.setVelocity(-rightFront.getVelocity());
                    leftBack.setVelocity(-leftBack.getVelocity());
                    rightBack.setVelocity(-rightBack.getVelocity());
                }
                while (leftFront.getVelocity() < 0 && currentVelocity < 0) {
                    leftFront.setVelocity(-leftFront.getVelocity());
                    rightFront.setVelocity(-rightFront.getVelocity());
                    leftBack.setVelocity(-leftBack.getVelocity());
                    rightBack.setVelocity(-rightBack.getVelocity());
                }
                leftFront.setVelocity(0);
                rightFront.setVelocity(0);
                leftBack.setVelocity(0);
                rightBack.setVelocity(0);
                leftFront.setMotorDisable();
                rightFront.setMotorDisable();
                leftBack.setMotorDisable();
                rightBack.setMotorDisable();
            } else {

                // controls the wheels
                leftFront.setMotorEnable();
                rightFront.setMotorEnable();
                leftBack.setMotorEnable();
                rightBack.setMotorEnable();
                leftFront.setPower((controlsOld.forwardMovement() + controlsOld.strafeMovement() + controlsOld.rotateMovement()) * driveClip);
                rightFront.setPower((controlsOld.forwardMovement() - controlsOld.strafeMovement() - controlsOld.rotateMovement()) * driveClip);
                leftBack.setPower((controlsOld.forwardMovement() - controlsOld.strafeMovement() + controlsOld.rotateMovement()) * driveClip);
                rightBack.setPower((controlsOld.forwardMovement() + controlsOld.strafeMovement() - controlsOld.rotateMovement()) * driveClip);

            }
        }
    }
}
