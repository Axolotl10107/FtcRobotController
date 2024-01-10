package org.firstinspires.ftc.teamcode.fy23.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.fy23.robot.units.PIDconsts;

public class RobotB {

    // Subsystems - include only and all the subsystems that this robot actually has
    public MecanumDrive drive;
    public FriendlyIMU imu;

    public PIDconsts pidConsts;

    public RobotB(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, "leftFront", "rightFront", "leftBack", "rightBack");

        pidConsts = new PIDconsts(0.023, 0.00, 0.00);
//        pidConsts = new PIDconsts(0, 0, 0);
        // TunablePID tuning for this robot

        imu = new FriendlyIMU(hardwareMap);
    }
}
