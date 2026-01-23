package org.firstinspires.ftc.teamcode.fy25.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.framework.adapters.DualDcMotorEx;
import org.firstinspires.ftc.teamcode.framework.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.firstinspires.ftc.teamcode.framework.util.TelemetrySingleton;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.IndyStarterBotScheme25;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.StarterBotState25;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;
import org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.Indexer;
import org.firstinspires.ftc.teamcode.fy25.subsystems.loader.Loader;

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
        DcMotorEx launchWheelMotor1 = hardwareMap.get(DcMotorEx.class, "launchWheelMotor");
        DcMotorEx launchWheelMotor2 = hardwareMap.get(DcMotorEx.class, "launchWheelMotor2");
        DualDcMotorEx launchWheelMotor = new DualDcMotorEx(launchWheelMotor1, launchWheelMotor2);
        launchWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // it doesn't want to work when i do it in robot roundhouse, i give up

        robot.drive.applyDTS( scaledDTS );

        if ( controlState.isBrake() ) {
            robot.drive.applyDTS( new DTS( 0, 0, 0 ) );
        }

//        if (controlState.getLauncherGateServoState() == LauncherGateServo.State.OPEN) {
//            robot.launchGateServo.open();
//        } else if (controlState.getLauncherGateServoState() == LauncherGateServo.State.CLOSED) {
//            robot.launchGateServo.close();
//        }
        ///  this has been replaced by an intake

        if (controlState.getLoaderState() == Loader.State.LOAD) {
            robot.loader.load();
        } else {
            robot.loader.pass();
        }

        if (controlState.getIndexState() == Indexer.State.NEXT) {
            robot.indexer.next();
            controlState.setIndexState(Indexer.State.READY);
        }

        if (controlState.isRunLaunchWheel()) {
            launchWheelMotor.setPower(1);
        } else {
            launchWheelMotor.setPower(0);
        }

        if (controlState.getIndexState() == Indexer.State.PREP) {
            robot.indexer.prepIntake();
        } else if (controlState.getIndexState() == Indexer.State.TO) {
            robot.indexer.goTo(robot.indexer.getIndex());
        }

        switch (controlState.getIntakeState()) {
            case RUNIN:
                robot.rotaryIntake.setState(RotaryIntake.State.RUNIN);
                break;
            case RUNOUT:
                robot.rotaryIntake.setState(RotaryIntake.State.RUNOUT);
                break;
            case STOPPED:
            default:
                robot.rotaryIntake.setState(RotaryIntake.State.STOPPED);
                robot.indexer.intake();
                controlState.setIndexState(Indexer.State.READY);
        }

        /// i haven't tested a lot of this because all my batteries died...
        /// sure hope it works and i don't have to spend hours redoing it all...
        /// . . .
        /// .  .  .
        /// how do i type an aggressive ellipsis
        /// •••
        /// ● ● ●
        /// yey
        /// wait i ruined the effect
        /// is that the right effect? maybe it's affect? what even is the difference? i know there is a difference but does anyone actually know which is which? maybe im just dumb.
        /// now that sounds like i was questioning whether the ellipsis came off right
        /// im going to bed ill commit this in the morning

        telemetry.addData("Indexer State", controlState.getIndexState());
        telemetry.addData("Indexer Goal", controlState.getIndexGoal());
        telemetry.addData("Index", controlState.getIndex());
        telemetry.addData("Index Pos", robot.indexer.getEncoder());
        telemetry.addData("Remaining Delta", robot.indexer.getRd());
        telemetry.addData("Intake State", robot.rotaryIntake.getState());
        telemetry.addData("Loader State", controlState.getLoaderState());
        telemetry.addData("Gate State", controlState.getLauncherGateServoState());
        telemetry.addData("Launch Wheel", controlState.isRunLaunchWheel());
        /// legend has it you can tell which subsystem was the most annoying to implement by looking at how many telemetry lines it has

        ///  I forgot to add these lines
        ///  that wasted so many hours
        /// how was telemetry ever working without updating it
        robot.update();
        telemetry.update();
    }
}
