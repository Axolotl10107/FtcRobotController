package org.firstinspires.ftc.teamcode.fy25.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.framework.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.framework.processors.TunablePID;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.firstinspires.ftc.teamcode.framework.util.TelemetrySingleton;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.IndyStarterBotScheme25;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.StarterBotState25;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate.LauncherGate;

@TeleOp(name="Starter Bot TeleOp (2025)", group="TeleOp25")
public class StarterBotTeleOp25 extends OpMode {
    Robot25 robot;
    IMUCorrector imuCorrector;
    IndyStarterBotScheme25 controlScheme;

    @Override
    public void init() {
        TelemetrySingleton.setInstance(telemetry);

        try {
            robot = RobotRoundhouse25.getRobotAuto(hardwareMap);
        } catch (RobotRoundhouse25.OldRobotException e) {
            throw new RuntimeException(e);
        }

        IMUCorrector.Parameters params = new IMUCorrector.Parameters( robot.imu, new TunablePID( robot.extendedParameters.hdgCorrectionPIDConsts ) );
        params.haveHitTargetToleranceDegrees = 0.1;
        params.hdgErrToleranceDegrees = 1.0;
        params.maxCorrectionPower = 0.1;
        params.turnPowerThreshold = 0.05;
        imuCorrector = new IMUCorrector( params );

        controlScheme = new IndyStarterBotScheme25( gamepad1, gamepad2 );
    }

    @Override
    public void loop() {

        double currentHeading = robot.imu.yaw();
        StarterBotState25 controlState = controlScheme.getState();

        // MecanumDrive
        DTS dts = controlState.getDts();
        telemetry.addData("Requested Drive", dts.drive);
        telemetry.addData("Requested Turn", dts.turn);
        telemetry.addData("Requested Strafe", dts.strafe);
        DTS correctedDTS = imuCorrector.correctDTS( dts );
        DTS normalizedDTS = correctedDTS.normalize();
        DTS scaledDTS = normalizedDTS.scale( controlState.getMaxDriveSpeed() );
        telemetry.addData("Actual Drive", scaledDTS.drive);
        telemetry.addData("Actual Turn", scaledDTS.turn);
        telemetry.addData("Actual Strafe", scaledDTS.strafe);
        robot.drive.applyDTS( scaledDTS );

        // Brake
        if ( controlState.isBrake() ) {
            robot.drive.applyDTS( new DTS( 0, 0, 0 ) );
        }

        robot.update();

        // telemetry
        telemetry.addData( "Max. Drive Power", controlState.getMaxDriveSpeed() );
        telemetry.addData( "Current Heading", currentHeading );
        telemetry.addData("Intake", controlState.getIntakeState());
        telemetry.addData("Lunch Gate", controlState.getLauncherGateState());
        telemetry.addData("Run Launch Wheel Requested?", controlState.isRunLaunchWheel());
        telemetry.addData("Launch Wheel Velocity", robot.launchWheel.getLaunchVelTarget());
        telemetry.addData("Distance", controlState.getDistance());
        robot.launchWheel.fixLaunchSpin(controlState.getDistance());

        if (controlState.isRunLaunchWheel()) {
            robot.launchWheel.spinUp();
        } else {
            robot.launchWheel.spinDown();
            robot.launchWheel.denyEntry();
        }

        if (controlState.getLauncherGateState() == LauncherGate.State.OPEN) {
            robot.launchGate.open();
        } else {
            robot.launchGate.close();
        }

        if (controlState.getIntakeState() == RotaryIntake.State.RUNIN) {
            robot.motorIntake.spinIn();
        } else {
            robot.motorIntake.stop();
        }

        telemetry.update();
    }
}
