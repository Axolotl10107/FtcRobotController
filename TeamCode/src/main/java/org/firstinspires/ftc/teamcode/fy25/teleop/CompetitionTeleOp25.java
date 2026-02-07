package org.firstinspires.ftc.teamcode.fy25.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.framework.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.firstinspires.ftc.teamcode.framework.util.TelemetrySingleton;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.IndyStarterBotScheme25;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.StarterBotState25;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate.LauncherGate;

@TeleOp(name="Competition TeleOp (2025)")
public class CompetitionTeleOp25 extends OpMode {
    Robot25 robot;
    IMUCorrector imuCorrector;
    IndyStarterBotScheme25 controlScheme;

    @Override
    public void init() {
        TelemetrySingleton.setInstance(telemetry);

        robot = new Robot25(RobotRoundhouse25.getRobotAParams(hardwareMap), hardwareMap);

        imuCorrector = new IMUCorrector( robot.extendedParameters.imuCorrectorParams );

        controlScheme = new IndyStarterBotScheme25( gamepad1, gamepad2 );
    }

    @Override
    public void loop() {

        // Initial Update
        double currentHeading = robot.imu.yaw();
        StarterBotState25 controlState = controlScheme.getState();


        // MecanumDrive
        DTS dts = controlState.getDts();
        DTS correctedDTS = imuCorrector.correctDTS( dts, robot.imu.yaw() );
        DTS normalizedDTS = correctedDTS.normalize();
        DTS scaledDTS = normalizedDTS.scale( controlState.getMaxDriveSpeed() );
        robot.drive.applyDTS( scaledDTS );

        telemetry.addData("Requested Drive", dts.drive);
        telemetry.addData("Requested Turn", dts.turn);
        telemetry.addData("Requested Strafe", dts.strafe);
        telemetry.addData("Actual Drive", scaledDTS.drive);
        telemetry.addData("Actual Turn", scaledDTS.turn);
        telemetry.addData("Actual Strafe", scaledDTS.strafe);

        if (controlState.isSquareUp()) {
            imuCorrector.squareUp();
        }


        // Brake (Note: since we're going through RRMecanumDrive, this is accel-limited! So it will stop gracefully.)
        if ( controlState.isBrake() ) {
            robot.drive.applyDTS( new DTS( 0, 0, 0 ) );
        }

        // LauncherWheel
//        robot.launchWheel.fixLaunchSpin(controlState.getDistance());

        if (controlState.isRunLaunchWheel()) {
            robot.launchWheel.spinUp();
        } else {
            robot.launchWheel.spinDown();
            if (controlState.getAllowEntry()) {
                robot.launchWheel.allowEntry();
            }
        }

        // LauncherGate
        if (controlState.getLauncherGateState() == LauncherGate.State.OPEN) {
            robot.launchGate.open();
        } else {
            robot.launchGate.close();
        }

        // RotaryIntake
        if (controlState.getIntakeState() == RotaryIntake.State.RUNIN) {
            robot.rotaryIntake.setState(RotaryIntake.State.RUNIN);
        } else {
            robot.rotaryIntake.setState(RotaryIntake.State.STOPPED);
        }

        // Telemetry
        telemetry.addData( "Max. Drive Power", controlState.getMaxDriveSpeed() );
        telemetry.addData( "Current Heading", currentHeading );
        telemetry.addData("Intake", controlState.getIntakeState());
        telemetry.addData("Lunch Gate", controlState.getLauncherGateState());
        telemetry.addData("Run Launch Wheel Requested?", controlState.isRunLaunchWheel());
        telemetry.addData("Launch Wheel Target RPM", robot.launchWheel.getLaunchVelTargetRPM());
        telemetry.addData("Launch Wheel Actual RPM", robot.launchWheel.getCurrentRPM());
        telemetry.addData("Launch Wheel State", robot.launchWheel.getState());
        telemetry.addData("Distance", controlState.getDistance());

        // Final Update
        robot.update();
        telemetry.update();
    }
}
