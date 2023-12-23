//MediumAutomaton 2023, for FTC22Robot / PowerNap
package org.firstinspires.ftc.teamcode.old_meddev;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class FTC22Robot {

    //Public Members (robot components)

    public RobotMap robotMap;

    public TieredElevator elevator;

    public ServoWrap armServo;//The actual ServoWrap objects
    public ServoWrap clawServo;

    public Manipulator manipulator;

//    public DistanceSensor leftOdo;
//    public DistanceSensor centerOdo;
//    public DistanceSensor rightOdo;

    //Private Members
//    private final float TPI = 100;//Unknown for now

    //Instantiate classes

    //Class Constructor
    public FTC22Robot(HardwareMap hardwareMap) {
        robotMap = new RobotMap(hardwareMap);
        elevator = new TieredElevator(robotMap.elevatorDrive);
        armServo = new ServoWrap(robotMap.armServo);
        clawServo = new ServoWrap(robotMap.clawServo);
        manipulator = new Manipulator(clawServo, armServo);

         robotMap.leftFront.setDirection(DcMotor.Direction.REVERSE);
        robotMap.rightFront.setDirection(DcMotor.Direction.FORWARD);
          robotMap.leftBack.setDirection(DcMotor.Direction.REVERSE);
         robotMap.rightBack.setDirection(DcMotor.Direction.FORWARD);

        elevator.setDirection(DcMotor.Direction.FORWARD);
        elevator.holdPosition();
        elevator.setPower(0);
        elevator.setCurrentPositionAsZero();
        elevator.setTieredMode();
    }

    //Methods

}
