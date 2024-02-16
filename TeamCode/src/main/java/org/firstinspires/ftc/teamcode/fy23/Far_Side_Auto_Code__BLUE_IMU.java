package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fy23.robot.old.RobotA;
import org.firstinspires.ftc.teamcode.fy23.robot.processors.IMUcorrector;
import org.firstinspires.ftc.teamcode.fy23.robot.units.DTS;

@Autonomous(name="IMU AutoCodeFarSide__BLUE", group="AutoCodeFarSide__RED")
public class Far_Side_Auto_Code__BLUE_IMU extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    // Motors for arm stuff
    private DcMotor armPivot = null;
    private DcMotor armExtend = null;

    private RobotA robot; // this contains your motors
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

    @Override
    public void runOpMode() {

        robot = new RobotA(hardwareMap);
        imuCorrector = new IMUcorrector(hardwareMap, robot.pidConsts);

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
        // about 83cm forward, 10cm to either side (for vision, not this...)

        targetPos = cmToTicks(119);
//        stopwatch.reset();
        desiredMotion = new DTS(0.5, 0, 0);
        while (robot.drive.getAvgEncoderPos() < targetPos) {
            corrected = imuCorrector.correctDTS(desiredMotion);
            robot.drive.applyDTS(0.5, 0, 0);
            telemetry.addData("current position", robot.drive.getAvgEncoderPos());
            telemetry.addData("current pos CM", ticksToCM(robot.drive.getAvgEncoderPos()));
            telemetry.update();
        }

        robot.drive.applyDTS(0,0,0);
        telemetry.addLine("Stopped");
        telemetry.update();

        sleep(1800);

        stopwatch.reset();
        desiredMotion = new DTS(0,0,-0.6);
        while (stopwatch.milliseconds() < 3800) {
            corrected = imuCorrector.correctDTS(desiredMotion);
            robot.drive.applyDTS(corrected);
        }

        robot.drive.applyDTS(0,0,0);
        telemetry.addLine("Stopped");
        telemetry.update();

        sleep(3800);






    }
}
