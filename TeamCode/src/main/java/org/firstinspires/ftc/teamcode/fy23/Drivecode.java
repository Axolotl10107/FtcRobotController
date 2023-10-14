package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name= "DriveCode", group = "Linear opmode")
public class Drivecode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

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

        // Wait for the game to start and this is where the driver hits play
        waitForStart();
        runtime.reset();

        // map drive functions to gamepad buttons
        forwardMovement = rightStickYLinear;
        strafeMovement = rightStickXLinear;
        rotateMovement = leftStickXLinear;

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
            drive = forwardMovement(1,1);
            strafe  =  strafeMovement(1,1);
            turn = rotateMovement(1,1);

            leftPower    = Range.clip(drive + strafe + turn - negative, -2, 2) ;
            rightPower   = Range.clip(drive - strafe - turn - negative, -2, 2) ;
            leftbackPower = Range.clip(drive - strafe + turn - negative, -2, 2) ;
            rightbackPower  = Range.clip(drive + strafe - turn - negative , -2, 2) ;

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftbackPower);
            rightBack.setPower(rightbackPower);


        }

        telemetry.update();
    }

    static double forwardMovement() {return 0;}
    static double strafeMovement() {return 0;}
    static double rotateMovement() {return 0;}

    static double rightStickYLinear(int gamepadNum, double scaling) {
        gamepad = (gamepadNum == 2) ? gamepad2 : gamepad1 ;
        return gamepad.right_stick_y()*scaling;
    }
    static double rightStickXLinear(int gamepadNum, double scaling) {
        gamepad = (gamepadNum == 2) ? gamepad2 : gamepad1 ;
        return gamepad.right_stick_x()*scaling;
    }
    static double triggerLinear(int gamepadNum, double scaling) {
        gamepad = (gamepadNum == 2) ? gamepad2 : gamepad1 ;
        return (gamepad.right_trigger()-gamepad.left_trigger())*scaling;
    }
    static double leftStickXLinear(int gamepadNum, double scaling) {
        gamepad = (gamepadNum == 2) ? gamepad2 : gamepad1 ;
        return gamepad.left_stick_x()*scaling;
    }
}

