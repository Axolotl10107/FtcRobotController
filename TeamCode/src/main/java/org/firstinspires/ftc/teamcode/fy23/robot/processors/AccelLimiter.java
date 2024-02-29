package org.firstinspires.ftc.teamcode.fy23.robot.processors;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.fy23.robot.Robot;

public class AccelLimiter {
    // Be consistent with your units! If maxAccel is in meters per second squared, pass in seconds to request().
    // If it's in, like, centimeters per millisecond squared, pass in CpM and milliseconds to request().

    double maxAccel = 1; // meters per second per second
    double maxDeltaVEachLoop = .5; // just to prevent big jumps (jerks) on long loops

    double _lastTime;
    double _oldVel;

    boolean initialized = false;

    public AccelLimiter(double argMaxAccel, double argMaxDeltaVEachLoop) {
        maxAccel = argMaxAccel;
        maxDeltaVEachLoop = argMaxDeltaVEachLoop;
    }

    public double request(double newVel, double currentVel, double currentTime) {
        if (initialized) {
            double loopTime = currentTime - _lastTime;
            double requestedDeltaVThisLoop = newVel - _oldVel;
            double targetDeltaVThisLoop = maxAccel * loopTime;
            double safeDeltaVThisLoop = Math.min(targetDeltaVThisLoop, maxDeltaVEachLoop);
            double actualDeltaVThisLoop = Range.clip(requestedDeltaVThisLoop, -safeDeltaVThisLoop, safeDeltaVThisLoop);
            double returnVel = _oldVel + actualDeltaVThisLoop;
            _oldVel = returnVel;
            _lastTime = currentTime;
//            System.out.println(String.format("_oldVel: {%f}", _oldVel));
//            System.out.println(String.format("returnVel: {%f}", returnVel));
//            System.out.println(String.format("_lastTime: {%f}", _lastTime));
            return returnVel;
        } else {
            System.out.println("Initializing");
            _oldVel = currentVel;
            _lastTime = currentTime;
            initialized = true;
            return _oldVel;
        }
    }

}
