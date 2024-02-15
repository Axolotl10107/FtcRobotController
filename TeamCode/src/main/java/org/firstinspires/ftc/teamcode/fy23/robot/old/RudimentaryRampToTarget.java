package org.firstinspires.ftc.teamcode.fy23.robot.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fy23.robot.old.RobotA;
import org.firstinspires.ftc.teamcode.fy23.robot.units.DTS;

// A stopgap solution! Don't use this. Use RoadRunner. We just don't have time to tune it this week...

/** Set a target relative to the starting position, in centimeters for forward/backward movement
 * and in degrees for heading. It will perform rudimentary acceleration control and move toward
 * the target. It does not currently support strafing and will simply turn to move sideways, but it
 * still returns its suggested power as a DTS. Its final action is to turn to the target heading.
 * It also currently only goes forwards. Maybe I'll change that later, but probably not because by
 * then I'll be using RoadRunner. */
public class RudimentaryRampToTarget {

    /**
     * This doesn't use RoadRunner. This is more rudimentary (and only exists because we don't
     * have time to tune for RR right now!), but I'm borrowing this unit from it.
     */
    private Pose2d targetPose;
    private Pose2d currentPose;

    /**
     * Maximum acceleration in centimeters per second
     */
    private double accel = 50;

    /**
     * Maximum acceleration in degrees per second
     */
    private double maxTurnAccel = 3;

    // Used to know how much time passes between getCurrentSuggestion() calls
    // because the timing of OpModes is inconsistent
    private ElapsedTime stopwatch = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double lastTime;
    private double lastPos;
    private double loopTime;
    private double loopMove;
    private double suggestedSpeed;
    private double newSpeed;
    private double totalDistance;
    private double timeToDecelerate;

    private boolean moveInProgress = false;
    //    private AnyRobot robot; // Using the interface has not gone well.
    private RobotA robot;

    public RudimentaryRampToTarget(RobotA argRobot) {
        robot = argRobot;
    }

    public RudimentaryRampToTarget(RobotA argRobot, Pose2d argTargetPose) {
        robot = argRobot;
        targetPose = argTargetPose;
    }

    public RudimentaryRampToTarget(RobotA argRobot, double argAccel) {
        robot = argRobot;
        accel = argAccel;
    }

    public RudimentaryRampToTarget(RobotA argRobot, Pose2d argTargetPose, double argAccel) {
        robot = argRobot;
        targetPose = argTargetPose;
        accel = argAccel;
    }

    public void setTargetPose(Pose2d argPose) {
        targetPose = argPose;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    /**
     * Set judiciously - it will always accelerate at this rate. (I told you that the
     * ramping is rudimentary.)
     */
    public void setAcceleration(double argAccel) {
        accel = argAccel;
    }

    public double getAcceleration() {
        return accel;
    }

    /**
     * Is it currently moving to the target?
     */
    public boolean moving() {
        return moveInProgress;
    }

    /**
     * Start moving towards the currently set target.
     */
    public void startMovement() {
        stopwatch.reset();
        moveInProgress = true;
        lastTime = 0;
        lastPos = 0;
        suggestedSpeed = 0;
        currentPose = new Pose2d(0, currentPose.getY(), robot.imu.yaw());
        totalDistance = targetPose.getY() - currentPose.getY();
        timeToDecelerate = (totalDistance / 2) / accel;
    }

    /**
     * Cancel the in-progress move. (This automatically happens when a move completes, so this is
     * only needed when the robot is actively moving.)
     */
    public void cancelMovement() {
        moveInProgress = false;
    }

    /**
     * Get the DTS suggested by the ramping algorithm. (It's a "suggestion" because you could run
     * it through processors.) */
    public DTS getCurrentSuggestion() {
        if (moving()) {
            loopTime = stopwatch.milliseconds() - lastTime; // How much time has passed
            loopMove = (((robot.drive.getAvgEncoderPos() - lastPos) / robot.TPR) * robot.wheelDiameter) / 10; // divide by 10 to get centimeters

            if (stopwatch.seconds() < 1 - (timeToDecelerate / totalDistance)) { // Can we still go...
                newSpeed = suggestedSpeed + (accel * loopTime) / 1000; // Dimensional analysis! How much can we accelerate now?
                // (3 cm/1 sec.) * (1 sec./1000 ms) * (loopTime/1 loop)
                suggestedSpeed = Math.max(newSpeed, robot.maxForwardSpeed); // Don't go too fast!
            } else { // ...or is it time to start decelerating now?
                newSpeed = suggestedSpeed - (accel * loopTime) / 1000; // Subtract instead of add
                suggestedSpeed = Math.min(newSpeed, 0); // Don't go backwards!
            }

            lastTime = stopwatch.milliseconds();
            lastPos = robot.drive.getAvgEncoderPos();
            currentPose = new Pose2d(0, currentPose.getY() + loopMove, robot.imu.yaw());
            return new DTS(suggestedSpeed, 0, 0);
        } else {
            return new DTS(0,0,0);
            // If the move is finished or cancelled, do nothing.
        }
    }
}
