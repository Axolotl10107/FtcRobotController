package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.units.DTS;

/** Represents a mecanum drive base, such as the goBILDA strafer. It is recommended to use {@link org.firstinspires.ftc.teamcode.fy23.robot.processors.DTSscaler}
 * with this. */
public class MecanumDrive {

    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;

    public MecanumDrive(HardwareMap hardwareMap, String lfName, String rfName, String lbName, String rbName) {
        leftFront = hardwareMap.get(DcMotorEx.class, lfName);
        rightFront = hardwareMap.get(DcMotorEx.class, rfName);
        leftBack = hardwareMap.get(DcMotorEx.class, lbName);
        rightBack = hardwareMap.get(DcMotorEx.class, rbName);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
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
}
