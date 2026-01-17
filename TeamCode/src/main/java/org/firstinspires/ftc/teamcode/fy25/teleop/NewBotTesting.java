package org.firstinspires.ftc.teamcode.fy25.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.firstinspires.ftc.teamcode.framework.util.TelemetrySingleton;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.IndyStarterBotScheme25;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.StarterBotState25;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;
import org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.Indexer;

@TeleOp(name="NewBotTesting")
public class NewBotTesting extends OpMode {
    Robot25 robot;

    IMUCorrector imuCorrector;

    IndyStarterBotScheme25 controlScheme;
    @Override
    public void init() {
        TelemetrySingleton.setInstance(telemetry);

        robot = new Robot25(RobotRoundhouse25.getRobotBParams(hardwareMap), hardwareMap);

        imuCorrector = new IMUCorrector( robot.extendedParameters.imuCorrectorParams );

        controlScheme = new IndyStarterBotScheme25( gamepad1, gamepad2 );
    }

    @Override
    public void loop() {
        StarterBotState25 controlState = controlScheme.getState();

        DTS dts = controlState.getDts();
        DTS correctedDTS = imuCorrector.correctDTS( dts, robot.imu.yaw() );
        DTS normalizedDTS = correctedDTS.normalize();
        DTS scaledDTS = normalizedDTS.scale( controlState.getMaxDriveSpeed() );
        robot.drive.applyDTS( scaledDTS );

        if ( controlState.isBrake() ) {
            robot.drive.applyDTS( new DTS( 0, 0, 0 ) );
        }

        if (controlState.getIndexState() == Indexer.State.TO) {
        }
    }
}
