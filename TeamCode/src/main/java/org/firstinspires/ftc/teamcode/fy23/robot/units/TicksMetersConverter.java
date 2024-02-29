package org.firstinspires.ftc.teamcode.fy23.robot.units;

import org.firstinspires.ftc.teamcode.fy23.robot.Robot;

public class TicksMetersConverter {

    private double _tpr;
    private double _wheelCircumference;

    public TicksMetersConverter(Robot robot) {
        _tpr = robot.TPR;
        _wheelCircumference = robot.wheelCircumference;
    }

    public TicksMetersConverter(double TPR, double wheelCircumference) {
        _tpr = TPR;
        _wheelCircumference = wheelCircumference;
    }

    public double toMeters(double ticks) {
        return (ticks / _tpr) * _wheelCircumference;
    }

    public double toTicks(double meters) {
        return (meters * _tpr) / _wheelCircumference;
    }


    public static double toMeters(double tpr, double wheelCircumference, double ticks) {
        return (ticks / tpr) * wheelCircumference;
    }

    public static double toTicks(double tpr, double wheelCircumference, double meters) {
        return (meters * tpr) / wheelCircumference;
    }

}
