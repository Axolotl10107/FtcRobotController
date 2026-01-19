package org.firstinspires.ftc.teamcode.fy25.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.framework.adapters.DualDcMotorEx;
import org.firstinspires.ftc.teamcode.framework.adapters.DualMotor;
import org.firstinspires.ftc.teamcode.framework.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.framework.units.DTS;
import org.firstinspires.ftc.teamcode.framework.util.TelemetrySingleton;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.IndyStarterBotScheme25;
import org.firstinspires.ftc.teamcode.fy25.ctlpad.StarterBotState25;
import org.firstinspires.ftc.teamcode.fy25.robots.Robot25;
import org.firstinspires.ftc.teamcode.fy25.robots.RobotRoundhouse25;
import org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.Indexer;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergateservo.LauncherGateServo;
import org.firstinspires.ftc.teamcode.fy25.subsystems.loader.Loader;

@TeleOp(name="NewBotTesting")
public class NewBotTesting extends OpMode {
    Robot25 robot;

    IMUCorrector imuCorrector;

    IndyStarterBotScheme25 controlScheme;
    double targetPos = 0;
    double ticksPerRevolution = 8192;
    double ticksPerIndex = ticksPerRevolution / 3;
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

        robot.drive.applyDTS( scaledDTS );

        if ( controlState.isBrake() ) {
            robot.drive.applyDTS( new DTS( 0, 0, 0 ) );
        }

        if (controlState.getLauncherGateServoState() == LauncherGateServo.State.OPEN) {
            robot.launchGateServo.open();
        } else if (controlState.getLauncherGateServoState() == LauncherGateServo.State.CLOSED) {
            robot.launchGateServo.close();
        }
        if (controlState.getLoaderState() == Loader.State.LOAD) {
            robot.loader.load();
        } else {
            robot.loader.pass();
        }
        if (controlState.getIndexState() == Indexer.State.TO) {
            robot.indexer.goTo(controlState.getIndexGoal());
            controlState.setIndexState(Indexer.State.READY);
        } else if (controlState.getIndexState() == Indexer.State.NEXT) {
            robot.indexer.next();
            controlState.setIndexState(Indexer.State.READY);
        } else if (controlState.getIndexState() == Indexer.State.PREP) {
            robot.indexer.prepIntake(controlState.getIndex());
        } else if (controlState.getIndexState() == Indexer.State.READY) {}

        if (controlState.isRunLaunchWheel()) {
            launchWheelMotor.setPower(1);
        } else {
            launchWheelMotor.setPower(0);
        }

        robot.indexer.update();

        telemetry.addData("Indexer State", controlState.getIndexState());
        telemetry.addData("Indexer Goal", controlState.getIndexGoal());
        telemetry.addData("Indexer Target", robot.indexer.getTarget());
        telemetry.addData("Index", controlState.getIndex());
        telemetry.addData("Index Pos", robot.indexer.getEncoder());
        telemetry.addData("Loader State", controlState.getLoaderState());
        telemetry.addData("Gate Servo", robot.launchGateServo.getPos());
        telemetry.addData("Gate State", controlState.getLauncherGateServoState());
        telemetry.addData("Launch Wheel", controlState.isRunLaunchWheel());


    }
}
