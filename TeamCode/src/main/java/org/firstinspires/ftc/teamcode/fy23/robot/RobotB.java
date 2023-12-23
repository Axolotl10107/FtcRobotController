package org.firstinspires.ftc.teamcode.fy23.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.units.PIDconsts;

public class RobotB {

    public MecanumDrive drive;
    public PIDconsts pidConsts;

    public RobotB(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, "leftFront", "rightFront", "leftBack", "rightBack");
        pidConsts = new PIDconsts(0.05, 0.05, 0.05);
        // TunablePID tuning for this robot
    }
}
