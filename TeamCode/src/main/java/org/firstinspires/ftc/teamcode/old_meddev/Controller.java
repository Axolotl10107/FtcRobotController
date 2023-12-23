//MediumAutomaton 2023, for FTC22Robot / PowerNap
package org.firstinspires.ftc.teamcode.old_meddev;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

//Elevator object
//Tiers could be constants holding the encoder position of their respective tiers?

public class Controller {
    //Gamepad containers
    public Gamepad driver;
    public Gamepad manipulator;

    private final ElapsedTime ddeb; //D-Pad and toggle button timeout
    private int ddebTimeout = 300; //change at runtime with setTimeout();

    //Public-facing fields
    public boolean clawToggle;//Claw Toggle
    public boolean clawOpen;//Claw Open
    public boolean clawClose;//Claw Close

    public boolean armToggle;//Arm Toggle
    public boolean armLeft;//Arm Left
    public boolean armRight;//Arm Right

    public double elevatorUp;//Elevator Up
    public double elevatorDown;//Elevator Down
    public boolean elevatorUpBool;
    public boolean elevatorDownBool;

    public boolean elevatorHigh;//Elevator High
    public boolean elevatorMedium;//Elevator Medium
    public boolean elevatorLow;//Elevator Low
    public boolean elevatorStack;//Elevator Stack
    public boolean elevatorGround;//Elevator Ground

    public boolean resetElevatorEncoder;//Reset Elevator Encoder
    public boolean storeElevatorHeightToTier;//Store Height to Tier
    public boolean elevatorMaxPowerUp;//Elevator Max Power Up
    public boolean elevatorMaxPowerDown;//Elevator Max Power Down

    public boolean slowReverse;//Slow Reverse
    public boolean slowForward;//Slow Forward
    public double analogReverse;//Analog Reverse
    public double analogForward;//Analog Forward

    public double turnAxis;//Turn Axis
    public double strafeAxis;//Strafe Axis

    public boolean driveMaxPowerUp;//Drive Max Power Up
    public boolean driveMaxPowerDown;//Drive Max Power Down
    public boolean turnStrafePowerUp;
    public boolean turnStrafePowerDown;

    //Constructor
    public Controller(Gamepad argDriver, Gamepad argManipulator) {
        driver = argDriver;
        manipulator = argManipulator;
        ddeb = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }



    //Polling Methods
    public void pollEverythingOpmode() {
        clawOpen = manipulator.a;
        clawClose = manipulator.x;

        armLeft = manipulator.y;
        armRight = manipulator.b;

        elevatorUp = manipulator.right_trigger;
        elevatorDown = manipulator.left_trigger;

        resetElevatorEncoder = manipulator.back;
        elevatorMaxPowerUp = manipulator.dpad_up;
        elevatorMaxPowerDown = manipulator.dpad_down;

        slowReverse = driver.left_bumper;
        slowForward = driver.right_bumper;
        analogReverse = driver.left_trigger;
        analogForward = driver.right_trigger;

        turnAxis = driver.left_stick_x;
        strafeAxis = driver.right_stick_x;

        if (ddeb.milliseconds() > ddebTimeout) {
            driveMaxPowerUp = driver.dpad_up;
            driveMaxPowerDown = driver.dpad_down;
            turnStrafePowerUp = driver.dpad_right;
            turnStrafePowerDown = driver.dpad_left;
            ddeb.reset();
        } else {
            driveMaxPowerUp = false;
            driveMaxPowerDown = false;
            turnStrafePowerUp = false;
            turnStrafePowerDown = false;
        }
    }

    public void pollSingle() {
        if (ddeb.milliseconds() > ddebTimeout) {
            clawToggle = driver.a;
            armToggle = driver.b;
        } else {
            clawToggle = false;
            armToggle = false;
        }

        elevatorUpBool = driver.y;
        elevatorDownBool = driver.x;

        elevatorHigh = driver.dpad_up;
        elevatorMedium = driver.dpad_left;
        elevatorLow = driver.dpad_right;
        elevatorStack = driver.dpad_down;

        resetElevatorEncoder = driver.back;
        storeElevatorHeightToTier = driver.start;

        turnAxis = driver.left_stick_x;
        strafeAxis = driver.right_stick_x;

        elevatorMaxPowerUp = manipulator.dpad_up;
        elevatorMaxPowerDown = manipulator.dpad_down;
        driveMaxPowerUp = manipulator.dpad_right;
        driveMaxPowerDown = manipulator.dpad_left;

        slowReverse = driver.left_bumper;
        analogReverse = driver.left_trigger;
        slowForward = driver.right_bumper;
        analogForward = driver.right_trigger;
    }

    public void setTimeout(int newTime) {
        ddebTimeout = newTime;
    }
}
