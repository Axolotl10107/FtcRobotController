package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fy23.processors.TunablePID;
import org.firstinspires.ftc.teamcode.fy23.robot.Robot24;
import org.firstinspires.ftc.teamcode.fy23.robot.RobotRoundhouse;
import org.firstinspires.ftc.teamcode.fy23.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

import kotlin.NotImplementedError;

@Autonomous(name="IMU AutoCodeFarSide__RED", group="AutoCodeFarSide__RED")
@Disabled
public class Far_Side_Auto_Code__RED_IMU extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    // Motors for arm stuff
    private DcMotor armPivot = null;
    private DcMotor armExtend = null;

    private Robot24 robot; // this contains your motors
    private IMUCorrector imuCorrector;

    DTS desiredMotion;
    DTS corrected;

    double targetPos;

    ElapsedTime stopwatch;

    // Please use RoadRunner instead!
    double ticksToCM(double ticks) {
//        return (ticks / robot.TPR) * robot.wheelCircumference;
        throw new NotImplementedError();
    }

    // Please use RoadRunner instead!
    double cmToTicks(double cm) {
//        return (cm * robot.TPR) / robot.wheelCircumference;
        throw new NotImplementedError();
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
    public void runOpMode() {

        robot = new Robot24(RobotRoundhouse.getRobotAParams(hardwareMap), hardwareMap);
        IMUCorrector.Parameters params = new IMUCorrector.Parameters(robot.imu, new TunablePID(robot.extendedParameters.hdgCorrectionPIDConsts));
        params.haveHitTargetToleranceDegrees = 0.1;
        params.hdgErrToleranceDegrees = 1.0;
        params.maxCorrectionPower = 0.1;
        params.turnPowerThreshold = 0.05;
        imuCorrector = new IMUCorrector(params);

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
