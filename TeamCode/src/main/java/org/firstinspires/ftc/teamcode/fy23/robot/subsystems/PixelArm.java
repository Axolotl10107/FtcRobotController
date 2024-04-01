package org.firstinspires.ftc.teamcode.fy23.robot.subsystems;

/** Represents the combination pivot (tilt, really) and elevator mechanism and allows both to be controlled by setting
 * their powers independently or by specifying a point on the planar region containing all possible points that this
 * mechanism can reach. (The latter has yet to be designed.) */
public interface PixelArm {

    class Parameters {
        /** Is this subsystem installed on this robot? */
        public boolean present;
        public String pivotMotorName;
        public String elevatorMotorName;
        public double maxPivotAccel;
        public double maxPivotDeltaVEachLoop;
        public double pivotMotorTPSAtHalfPower;
        public double pivotMotorTPSAtFullPower;
        public int pivotUpperLimit;
        public int pivotLowerLimit;
        public double maxElevatorAccel;
        public double maxElevatorDeltaVEachLoop;
        public int elevatorUpperLimit;
        public int elevatorLowerLimit;
        public double elevatorMotorTPSAtHalfPower;
        public double elevatorMotorTPSAtFullPower;
    }

    void setPivotPower(double power);
    double getPivotPower();

    void setElevatorPower(double power);
    double getElevatorPower();

    /** Called by robot.update(). You do not need to call this method. */
    void update();

}
