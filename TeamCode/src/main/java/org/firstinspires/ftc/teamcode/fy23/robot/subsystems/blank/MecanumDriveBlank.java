package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.fy23.units.DTS;

/** A blank implementation of {@link MecanumDrive} that does nothing. */
public class MecanumDriveBlank implements MecanumDrive {

    @Override
    public void applyDTS(DTS dts) {

    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {

    }

    @Override
    @Deprecated
    public int getAvgEncoderPos() {
        return 0;
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {

    }

    @Override
    public void update() {

    }

}
