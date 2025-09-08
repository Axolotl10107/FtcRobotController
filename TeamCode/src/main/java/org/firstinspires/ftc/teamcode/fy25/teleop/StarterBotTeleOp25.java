package org.firstinspires.ftc.teamcode.fy25.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.framework.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.framework.processors.TunablePID;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.firstinspires.ftc.teamcode.framework.util.TelemetrySingleton;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.IndyStarterBotScheme25;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.StarterBotState25;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;

@TeleOp(name="Starter Bot TeleOp (2025)", group="TeleOp25")
public class StarterBotTeleOp25 extends OpMode {

    Robot25 robot;
    IMUCorrector imuCorrector;
    IndyStarterBotScheme25 controlScheme;
    double maxDrivePower = 1.0;

    @Override
    public void init() {
        try {
            robot = new Robot25( RobotRoundhouse25.getParamsAuto( hardwareMap ), hardwareMap );
        } catch (Robot25.InvalidDeviceClassException | RobotRoundhouse25.OldRobotException e) {
            throw new RuntimeException(e);
        }

        IMUCorrector.Parameters params = new IMUCorrector.Parameters( robot.imu, new TunablePID( robot.extendedParameters.hdgCorrectionPIDConsts ) );
        params.haveHitTargetToleranceDegrees = 0.1;
        params.hdgErrToleranceDegrees = 1.0;
        params.maxCorrectionPower = 0.1;
        params.turnPowerThreshold = 0.05;
        imuCorrector = new IMUCorrector( params );

        controlScheme = new IndyStarterBotScheme25( gamepad1, gamepad2 );

        TelemetrySingleton.setInstance(telemetry);
    }

    @Override
    public void loop() {
        telemetry.addData("Place", "Loop start");
        telemetry.update();

        double currentHeading = robot.imu.yaw();
        StarterBotState25 controlState = controlScheme.getState();

        telemetry.addData("Place", "Got controls state");
        telemetry.update();

        // MecanumDrive
        DTS correctedDTS = imuCorrector.correctDTS( controlState.getDts() );
        DTS normalizedDTS = correctedDTS.normalize();
        DTS scaledDTS = normalizedDTS.scale( controlState.getMaxDriveSpeed() );
        robot.drive.applyDTS( scaledDTS );

        telemetry.addData("Place", "Handled DTS");
        telemetry.update();
//        telemetry.addData("Square up?", controlState.isSquareUp());
//        telemetry.update();
//
//        // IMUcorrector - square up
//        if ( controlState.isSquareUp() ) {
//            imuCorrector.squareUp();
//        }

        telemetry.addData("Place", "Handled square up");
        telemetry.update();

        // Brake
        if ( controlState.isBrake() ) {
            robot.drive.applyDTS( new DTS( 0, 0, 0 ) );
        }

        telemetry.addData("Place", "Handled brake");
        telemetry.update();

        // Manipulator
//        robot.claw.setState( controlState.getClawState() );

        telemetry.addData("Place", "Handled claw");
        telemetry.update();

        // PixelArm
//        robot.arm.setPivotPower( controlState.getArmMovement() );
//        robot.arm.setElevatorPower( controlState.getElevatorMovement() );

        telemetry.addData("Place", "Handled arm");
        telemetry.update();

        robot.update();

        telemetry.addData("Place", "Ran robot.update()");
        telemetry.update();

        // telemetry
        telemetry.addData( "Max. Drive Power", controlState.getMaxDriveSpeed() );
        telemetry.addData( "Current Heading", currentHeading );
        telemetry.update();
    }
}
