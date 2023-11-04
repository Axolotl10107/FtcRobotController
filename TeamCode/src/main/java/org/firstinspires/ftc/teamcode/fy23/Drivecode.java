package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.fy23.controls.GamepadInterface;

@TeleOp(name= "DriveCode", group = "Linear opmode")
public class Drivecode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    //Manipulator stuffs
    TouchSensor touch;
    DcMotor armPivot;
    DcMotor armExtend;
    Servo clawServo;

    double upPower = 0;
    double downPower = 0;
    //End manipulator stuffs

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //Manipulator stuffs
        touch = hardwareMap.get(TouchSensor.class, "pivotLowerLimit");
        armPivot = hardwareMap.get(DcMotor.class, "armPivot");
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        armExtend.setDirection(DcMotorSimple.Direction.REVERSE);

//        armPivot.setTargetPosition(armPivot.getCurrentPosition());
//        armPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armPivot.setPower(0);
        //End manipulator stuffs

        // Wait for the game to start and this is where the driver hits play
        waitForStart();
        runtime.reset();
//we coment stuff out so it will actually go to the robot
        // map drive functions to gamepad buttons Sorry
//        forwardMovement = rightStickYLinear;
//        strafeMovement = rightStickXLinear;
//        rotateMovement = leftStickXLinear;
        GamepadInterface gamePad = new GamepadInterface() {
            @Override
            public double forwardMovement() {
                return rightStickYLinear(1,1);
            }

            @Override
            public double strafeMovement() {
                return rightStickXLinear(1,1);
            }

            @Override
            public double rotateMovement() {
                return leftStickXLinear(1,1);
            }

            @Override
            public double armMovement() {
                return 0;
            }

            @Override
            public double elevatorMovement() {
                return 0;
            }

            @Override
            public double clawOpen() {
                return 0;
            }

            @Override
            public double clawClose() {
                return 0;
            }
        };

        while (opModeIsActive()) {

            double leftPower;
            double rightPower;
            double leftbackPower;
            double rightbackPower;

            // POV Mode uses left stick to go forward, and right stick to strafe.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive;
            double negative = 0;
            double strafe;
            double turn;

            // crappy way of switching control schemes, will delete later
            // boolean combinedTranslationControls = true;
            // if (combinedTranslationControls) {
            //     drive = -gamepad1.right_stick_y*.75;
            //     negative = 0;
            //     strafe  =  gamepad1.right_stick_x*.75;
            //     turn = gamepad1.left_stick_x*.75;
            // } else {
            //     drive = gamepad1.right_trigger;
            //     negative = gamepad1.left_trigger;
            //     strafe  =  gamepad1.right_stick_x;
            //     turn = gamepad1.left_stick_x;
            // }

// i am removing the 1,1 in the () you will need to add them back probably
            drive = gamePad.forwardMovement();
            strafe = gamePad.strafeMovement();
            turn = gamePad.rotateMovement();

            leftPower = Range.clip(drive + strafe + turn - negative, -1, 1);
            rightPower = Range.clip(drive - strafe - turn - negative, -1, 1);
            leftbackPower = Range.clip(drive - strafe + turn - negative, -1, 1);
            rightbackPower = Range.clip(drive + strafe - turn - negative, -1, 1);

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftbackPower);
            rightBack.setPower(rightbackPower);

            telemetry.update();

            //Manipulator stuffs
            if (0 < armExtend.getCurrentPosition()) {
                downPower = gamepad2.left_trigger;
            } else {
                downPower = 0;
            }

            if (armExtend.getCurrentPosition() < 2000) {
                upPower = gamepad2.right_trigger;
            } else {
                upPower = 0;
            }

            armExtend.setPower((upPower - downPower) * 0.6);

            //claw
            if (gamepad2.x) {
                clawServo.setPosition(0.05);//Opens claw
            } else if (gamepad2.a) {
                clawServo.setPosition(0.19);//Closes claw
            }

            armPivot.setPower(gamepad2.right_stick_x);

            telemetry.addData("Encoder Position", armExtend.getCurrentPosition());
            //End manipulator stuffs
        }
    }

    //
    double rightStickYLinear(int gamepad, double scaling) {
        if (gamepad == 2) {
            return gamepad2.right_stick_y * scaling;
        } else {
            return gamepad1.right_stick_y * scaling;
        }
    }

    double rightStickXLinear(int gamepad, double scaling) {
        if (gamepad == 2) {
            return gamepad2.right_stick_x * scaling;
        } else {
            return gamepad1.right_stick_x * scaling;
        }
    }

    double leftStickXLinear(int gamepad, double scaling) {
        if (gamepad == 2) {
            return gamepad2.left_stick_x * scaling;
        } else {
            return gamepad1.left_stick_x * scaling;
        }
    }

    double rightStickYExponential(int gamepad, double scaling) {
        if (gamepad == 2) {
            return -Math.pow(gamepad2.right_stick_y, scaling);
        } else {
            return -Math.pow(gamepad1.right_stick_y, scaling);
        }
    }
}
// Probably need to add a } if the above code commented out is used!!!!

