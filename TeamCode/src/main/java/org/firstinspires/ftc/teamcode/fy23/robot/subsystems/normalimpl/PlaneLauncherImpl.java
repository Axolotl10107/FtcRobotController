package org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.PlaneLauncher;

public class PlaneLauncherImpl implements PlaneLauncher {

    private Servo servo;

    public PlaneLauncherImpl(Parameters parameters, HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "planeservo");
    }

    @Override
    public void launch() {
        servo.setPosition(1);
    }

    @Override
    public void handleAutoRetract() {
        if (servo.getPosition() > 0.01) {
            servo.setPosition(0);
        }
    }
}
