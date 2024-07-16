package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.robot.old.RobotA;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUcorrector;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

@Autonomous(name="IMU AutoCodeFarSide__RED", group="AutoCodeFarSide__RED")
public class Far_Side_Auto_Code__RED_IMU extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    // Motors for arm stuff
    private DcMotor armPivot = null;
    private DcMotor armExtend = null;

    private Robot robot; // this contains your motors
    private IMUcorrector imuCorrector;

    DTS desiredMotion;
    DTS corrected;

    double targetPos;

    ElapsedTime stopwatch;

    double ticksToCM(double ticks) {
        return (ticks / robot.TPR) * robot.wheelCircumference;
    }

    double cmToTicks(double cm) {
        return (cm * robot.TPR) / robot.wheelCircumference;
    }

    private int getAvgEncoderPos() {
        return (
                robot.drive.leftFront.getCurrentPosition() +
                robot.drive.rightFront.getCurrentPosition() +
                robot.drive.leftBack.getCurrentPosition() +
                robot.drive.rightBack.getCurrentPosition()
        ) / 4;
    }

    @Override
    public void runOpMode() {

        robot = new Robot(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
        IMUcorrector.Parameters params = new IMUcorrector.Parameters();
        params.haveHitTargetTolerance = 0.1;
        params.hdgErrTolerance = 1.0;
        params.maxCorrection = 0.1;
        params.turnThreshold = 0.05;
        params.imu = robot.imu;
        params.pid = new TunablePID(robot.hdgCorrectionPIDconsts);
        params.errorSampleTimer = new ElapsedTime();
        params.errorSampleDelay = 1150;
        imuCorrector = new IMUcorrector(params);

        stopwatch = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        robot.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        telemetry.addData("Status", "Ready for Initialisation");

        armPivot = hardwareMap.get(DcMotor.class, "armPivot");
        armPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        armPivot.setTargetPosition(armPivot.getCurrentPosition());


        //Code for Pivot the arm

        armPivot.setTargetPosition(500);
        armPivot.setPower(.4);
        sleep(200);

        armPivot.setPower(0.1);

        sleep(1000);

        //Code for moving forward

        targetPos = cmToTicks(119);
        stopwatch.reset();
        desiredMotion = new DTS(0.5, 0, 0);
        while (getAvgEncoderPos() < targetPos) {
            corrected = imuCorrector.correctDTS(desiredMotion);
            robot.drive.applyDTS(new DTS(0.5, 0, 0));
        }

        robot.drive.applyDTS(new DTS(0,0,0));

        sleep(1800);

        stopwatch.reset();
        desiredMotion = new DTS(0,0,0.6);
        while (stopwatch.milliseconds() < 3800) {
            corrected = imuCorrector.correctDTS(desiredMotion);
            robot.drive.applyDTS(corrected);
        }

        sleep(1750);








    }
}
