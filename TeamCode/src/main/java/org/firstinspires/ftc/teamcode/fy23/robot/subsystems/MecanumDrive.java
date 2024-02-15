package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.units.DTS;

/** Represents a mecanum drive base, such as the goBILDA strafer. It is recommended to use {@link org.firstinspires.ftc.teamcode.fy23.robot.processors.DTSscaler}
 * with this. */
public class MecanumDrive {

    public static class Parameters {
        public boolean present;

        public String leftFrontName;
        public DcMotor.Direction leftFrontDirection;

        public String rightFrontName;
        public DcMotor.Direction rightFrontDirection;

        public String leftBackName;
        public DcMotor.Direction leftBackDirection;

        public String rightBackName;
        public DcMotor.Direction rightBackDirection;

        public DcMotor.RunMode runMode;
        public DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    }

    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;

    public MecanumDrive(Parameters parameters, HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, parameters.leftFrontName);
        rightFront = hardwareMap.get(DcMotorEx.class, parameters.rightFrontName);
        leftBack = hardwareMap.get(DcMotorEx.class, parameters.leftBackName);
        rightBack = hardwareMap.get(DcMotorEx.class, parameters.rightBackName);

        leftFront.setDirection(parameters.leftFrontDirection);
        rightFront.setDirection(parameters.rightFrontDirection);
        leftBack.setDirection(parameters.leftBackDirection);
        rightBack.setDirection(parameters.rightBackDirection);

        try {
            setMode(parameters.runMode);
        } catch (Exception x) {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // if it wasn't set, pick a default
        }

        try {
            setZeroPowerBehavior(parameters.zeroPowerBehavior);
        } catch (Exception x) {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    /** Takes these components as motor powers */
    public void applyDTS(double drive, double turn, double strafe) {
        leftFront.setPower(drive - turn + strafe);
        rightFront.setPower(drive + turn - strafe);
        leftBack.setPower(drive - turn - strafe);
        rightBack.setPower(drive + turn + strafe);
    }

    /** Takes these components as motor powers */
    public void applyDTS(DTS dts) { // function overloading
        applyDTS(dts.drive, dts.turn, dts.strafe);
    }

    public double getAvgEncoderPos() {
        return (
                leftFront.getCurrentPosition() +
                rightFront.getCurrentPosition() +
                leftBack.getCurrentPosition() +
                rightBack.getCurrentPosition()
        ) / 4;
    }

    public void setMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
    }

    public void setVelocity(double velocity) {
        leftFront.setVelocity(velocity);
        rightFront.setVelocity(velocity);
        leftBack.setVelocity(velocity);
        rightBack.setVelocity(velocity);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);

    }
}
