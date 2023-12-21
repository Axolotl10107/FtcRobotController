package org.firstinspires.ftc.teamcode.fy23;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMUdrive {
    /*
    I've tried to note this where necessary, but I'll put a big notice here. The Orientation
    returned by the BNO055IMU is right-handed, so positive yaw (turn) is counterclockwise. I have
    made the turning power the same - positive power turns counterclockwise. Please keep this in
    mind.
     */

    //configuration
    private double hdgErrPower = 0.1;
    // applyTurn = turn + (hdgErrPower * headingError);
    private double maxTotalCorrection = 0.5;
    private float hdgErrThreshold = (float) 0.1;
    // minimum actionable heading error
    private double turnThreshold = 0.05;
    // how much you must be turning for heading maintenance to temporarily stop

    //initialization
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    public BNO055IMU imu; //our control hubs should have this type
    private BNO055IMU.Parameters imuParams; //stores configuration stuff for the IMU

    Orientation orientation;
    float heading;

    double drive;
    double turn;
    double lastTurn = 0;
    double strafe;

    float targetHeading;
    float headingError;

    double applyDrive;
    double applyTurn;
    double applyStrafe;
    double GCD; // greatest common denominator

    private Telemetry telemetry;

    public IMUdrive(HardwareMap hardwareMap, Telemetry argTelemetry) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        imuParams = new BNO055IMU.Parameters();
        imuParams.calibrationDataFile = "BNO055IMUCalibration.json";
        //see "SensorBNO055IMUCalibration" example

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);

        telemetry = argTelemetry;
    }


    //internal methods
    private void updateOrientation() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        heading = orientation.thirdAngle;
        headingError = targetHeading - heading; //positive turn is counterclockwise!
    }

    private void applyAllPowers() {
        applyDrive = drive;
        applyStrafe = strafe;

//        if (Math.abs(headingError) > hdgErrThreshold) {
//            applyTurn = turn + Range.clip((headingError * hdgErrPower), -maxTotalCorrection, maxTotalCorrection);
//        }
//        else {
//            applyTurn = turn;
//        }

        if (Math.abs(turn) > turnThreshold) {
            applyTurn = turn;
        }
        else {
            applyTurn = Range.clip((headingError * hdgErrPower), -maxTotalCorrection, maxTotalCorrection);
        }

        if (Math.abs(turn) < turnThreshold && lastTurn > turnThreshold) {
            targetHeading = heading;
        }
        lastTurn = Math.abs(turn);

        GCD = Math.max(Math.abs(applyDrive) + Math.abs(applyTurn) + Math.abs(applyStrafe), 1);

        //Positive turn is counterclockwise!
        leftFront.setPower((applyDrive - applyTurn + applyStrafe) / GCD);
        rightFront.setPower((applyDrive + applyTurn - applyStrafe) / GCD);
        leftBack.setPower((applyDrive - applyTurn - applyStrafe) / GCD);
        rightBack.setPower((applyDrive + applyTurn + applyStrafe) / GCD);
    }

    private void constructTelemetry() {
        telemetry.addData("Correcting Heading", (headingError > hdgErrThreshold));
        telemetry.addLine("--------------------------------");
        telemetry.addData("leftFront Power", leftFront.getPower());
        telemetry.addData("rightFront Power", rightFront.getPower());
        telemetry.addData("leftBack Power", leftBack.getPower());
        telemetry.addData("rightBack Power", rightBack.getPower());
        telemetry.addLine("--------------------------------------");
        telemetry.addData("Requested drive", drive);
        telemetry.addData("Applied drive", applyDrive);
        telemetry.addLine("--------------------------------------");
        telemetry.addData("Requested turn", turn);
        telemetry.addData("Applied turn", applyTurn);
        telemetry.addData("Error Correction", headingError * hdgErrPower);
        telemetry.addLine("--------------------------------------");
        telemetry.addData("Requested strafe", strafe);
        telemetry.addData("Applied strafe", applyStrafe);
        telemetry.addLine("--------------------------------------");
        telemetry.addData("GCD", GCD);
        telemetry.addLine("--------------------------------------");
        telemetry.addData("Current Heading", heading);
        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("Heading Error", headingError);
        telemetry.addData("Error Threshold", hdgErrThreshold);
    }


    //public methods
    public void update() {
        updateOrientation();
        applyAllPowers();
        constructTelemetry();
    }

    public void setDrivePower(double argDrive) {
        drive = argDrive;
    }

    public void setTurnPower(double argTurn) {
        turn = argTurn; //Positive turn is counterclockwise!
    }

    public void setTargetHeading(float argTarget) {
        targetHeading = argTarget;
    }

    public float getTargetHeading() {
        return targetHeading;
    }

    public void setStrafePower(double argStrafe) {
        strafe = argStrafe;
    }

    public void setAllPowers(double argDrive, double argTurn, double argStrafe) {
        setDrivePower(argDrive);
        setTurnPower(argTurn);
        setStrafePower(argStrafe);
    }
}
