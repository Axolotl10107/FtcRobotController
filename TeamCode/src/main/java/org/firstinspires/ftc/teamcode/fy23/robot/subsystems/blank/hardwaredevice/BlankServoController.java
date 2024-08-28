package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.hardwaredevice;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import org.jetbrains.annotations.NotNull;

public class BlankServoController extends BlankHardwareDevice implements ServoControllerEx {
    @Override
    public void setServoPwmRange(int servo, @NonNull @NotNull PwmControl.PwmRange range) {

    }

    @NonNull
    @NotNull
    @Override
    public PwmControl.PwmRange getServoPwmRange(int servo) {
        return new PwmControl.PwmRange(0, 0);
    }

    @Override
    public void setServoPwmEnable(int servo) {

    }

    @Override
    public void setServoPwmDisable(int servo) {

    }

    @Override
    public boolean isServoPwmEnabled(int servo) {
        return false;
    }

    @Override
    public void setServoType(int servo, ServoConfigurationType servoType) {

    }

    @Override
    public void pwmEnable() {

    }

    @Override
    public void pwmDisable() {

    }

    @Override
    public PwmStatus getPwmStatus() {
        return PwmStatus.DISABLED;
    }

    @Override
    public void setServoPosition(int servo, double position) {

    }

    @Override
    public double getServoPosition(int servo) {
        return 0;
    }
}
