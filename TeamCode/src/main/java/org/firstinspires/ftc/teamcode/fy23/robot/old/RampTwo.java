package org.firstinspires.ftc.teamcode.fy23.robot.old;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** This is broken! Please follow the progress on {@link org.firstinspires.ftc.teamcode.fy23.processors.AccelLimiter} instead. */
public class RampTwo {

    private double deltaVel;
    private double accel;
    private double loopTime;
    private double lastTime;
    private double currentTime;
    private double currentVel;
    private double newVel;
    public double remainingDistance;
    private double totalDistance;
    public double timeToDecel;
    public double accelToStop;
//    private double currentPos;

    private double maxAccel = 1;
    private double targetVel = 3;
    private double maxDeltaVel = 0.01;
    private double minVel = 0.1;
    private double targetPos = 100;

    private double maxVel;

    private ElapsedTime stopwatch;

    public RampTwo(double argMaxAccel, double argTargetVel, double argMaxDeltaVel, double argMinVel, double argTargetPos, double argCurrentPos) {
        maxAccel = argMaxAccel;
        targetVel = argTargetVel;
        maxDeltaVel = argMaxDeltaVel;
        minVel = argMinVel;
        targetPos = argTargetPos;

        stopwatch = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

//        currentPos = 0;
        currentTime = 0;
        lastTime = 0;
        currentVel = 0;

        totalDistance = targetPos - argCurrentPos;

        maxVel = targetVel;
    }

    public double getSuggestionAtPos(double currentPos) {
        if (currentPos < targetPos) {
            if (targetVel >= currentVel) {
                accel = maxAccel;
            } else {
                accel = -maxAccel;
            }

//            currentPos = ticksToCM(robot.drive.getAvgEncoderPos());
            remainingDistance = totalDistance - currentPos;

            timeToDecel = remainingDistance / currentVel;
            accelToStop = currentVel / timeToDecel;
            if (accelToStop >= Math.abs(accel) - 0.1) {
                targetVel = 0;
            }

            currentTime = stopwatch.milliseconds();
            loopTime = currentTime - lastTime;
            loopTime = loopTime / 1000; // convert to seconds to be consistent with accel

            deltaVel = accel * loopTime;
            deltaVel = Range.clip(deltaVel, -maxDeltaVel, maxDeltaVel);
            newVel = currentVel + deltaVel;
            newVel = Range.clip(newVel, minVel, maxVel);

            lastTime = currentTime;
            currentVel = newVel;
            return currentVel;
        } else {
            return 0;
        }
    }
}
