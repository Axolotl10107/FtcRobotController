package org.firstinspires.ftc.teamcode.fy25.robots;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.framework.processors.IMUCorrector;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMUBlank;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMUImpl;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntakeBlank;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntakeImpl;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDriveBlank;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDriveImpl;
import org.firstinspires.ftc.teamcode.fy25.subsystems.artifactsensor.ArtifactSensor;
import org.firstinspires.ftc.teamcode.fy25.subsystems.artifactsensor.ArtifactSensorBlank;
import org.firstinspires.ftc.teamcode.fy25.subsystems.artifactsensor.ArtifactSensorImpl;
import org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.Indexer;
import org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.IndexerBlank;
import org.firstinspires.ftc.teamcode.fy25.subsystems.indexer.IndexerImpl;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergateservo.LauncherGateServo;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergateservo.LauncherGateServoBlank;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergateservo.LauncherGateServoImpl;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel.LauncherWheelImpl;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheelsimple.LauncherWheelSimple;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheelsimple.LauncherWheelSimpleBlank;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheelsimple.LauncherWheelSimpleImpl;
import org.firstinspires.ftc.teamcode.fy25.subsystems.loader.Loader;
import org.firstinspires.ftc.teamcode.fy25.subsystems.loader.LoaderBlank;
import org.firstinspires.ftc.teamcode.fy25.subsystems.loader.LoaderImpl;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motifreader.MotifReader;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motifreader.MotifReaderBlank;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motifreader.MotifReaderImpl;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer.MotorIndexer;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer.MotorIndexerBlank;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motorindexer.MotorIndexerImpl;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motorintake.MotorIntake;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motorintake.MotorIntakeBlank;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motorintake.MotorIntakeImpl;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate.*;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel.LauncherWheel;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel.LauncherWheelBlank;

/**Represents a complete robot consisting of up to 5 subsystems.
 * Basically, constructing this will construct all of the subsystems for you,
 * which in turn do all of the low-level hardware setup work.
 * <p>
 * Call methods of subsystems to perform actions. For example, to open the claw:
 * <code>robot.claw.setState(Claw.State.OPEN);</code>
 * Or to start driving forward:
 * <code>robot.drive.applyDTS(new DTS(1, 0, 0));</code>
 * <p>
 * Programmers who are used to working directly with the SDK to write complete
 * TeleOps might appreciate turning about 120 lines of boilerplate code and a
 * different OpMode for each robot into the following line in a single OpMode:
 * <code>Robot25 robot = new Robot25(RobotRoundhouse25.getParamsAuto(), hardwareMap);</code>
 * Or, new this year:
 * <code>Robot25 robot = RobotRoundhouse25.getRobotAuto(hardwareMap);</code>
 * <p>
 * See {@link RobotRoundhouse25} to find {@link Parameters} to pass in here. */
public class Robot25 {

    /** Robot-specific parameters that are not used directly by the Robot but
     * by external things, usually processors. */
    public static class ExtendedParameters {
        /** Used by IMUCorrector.
         * We'll use this as our example. The Robot doesn't need this; only
         * IMUCorrector does. But it's Robot-specific, so it has to be a Robot
         * parameter. You can leave this un-set (or, rather, at the default
         * value), but then IMUCorrector won't do anything on this Robot. */
//        public PIDConsts hdgCorrectionPIDConsts = new PIDConsts(0, 0, 0, 0);
        public IMUCorrector.Parameters imuCorrectorParams;
    }

    /** Subsystems are encapsulated in this class, so their Parameters are too. */
    public static class Parameters {
        public Parameters(
                ExtendedParameters extendedParameters,
                RRMecanumDrive.Parameters driveParameters,
                FriendlyIMU.Parameters imuParameters,

                LauncherWheelSimple.Parameters launchWheelSimpleParams,
                LauncherWheel.Parameters launchWheelParams,
                LauncherGate.Parameters launchGateParams,
                LauncherGateServo.Parameters launchGateServoParams,
                MotorIntake.Parameters motorIntakeParams,
                RotaryIntake.Parameters rotaryIntakeParams,

                Indexer.Parameters indexerParameters,
                MotorIndexer.Parameters motorIndexerParameters,
                Loader.Parameters loaderParameters,

                ArtifactSensor.Parameters artifactSensorParameters,

                MotifReader.Parameters motifReaderParameters
        ) {
            // Every season
            this.extendedParameters = extendedParameters;
            this.driveParameters = driveParameters;
            this.imuParameters = imuParameters;

            // This season
            this.launchWheelSimpleParams = launchWheelSimpleParams;
            this.launchWheelParams = launchWheelParams;
            this.launchGateParams = launchGateParams;
            this.launcherGateServoParams = launchGateServoParams;
            this.motorIntakeParams = motorIntakeParams;
            this.rotaryIntakeParams = rotaryIntakeParams;

            this.motorIndexerParameters = motorIndexerParameters;
            this.indexerParameters = indexerParameters;
            this.loaderParameters = loaderParameters;

            this.artifactSensorParameters = artifactSensorParameters;

            this.motifReaderParameters = motifReaderParameters;
        }

        final ExtendedParameters extendedParameters;
        final RRMecanumDrive.Parameters driveParameters;

        final FriendlyIMU.Parameters imuParameters;
        final LauncherWheelSimple.Parameters launchWheelSimpleParams;
        final LauncherWheel.Parameters launchWheelParams;
        final LauncherGate.Parameters launchGateParams;

        final LauncherGateServo.Parameters launcherGateServoParams;
        final MotorIntake.Parameters motorIntakeParams;
        final RotaryIntake.Parameters rotaryIntakeParams;
        final MotorIndexer.Parameters motorIndexerParameters;
        final Indexer.Parameters indexerParameters;
        final Loader.Parameters loaderParameters;
        final ArtifactSensor.Parameters artifactSensorParameters;

        final MotifReader.Parameters motifReaderParameters;

    }

    public final ExtendedParameters extendedParameters;

    public final RRMecanumDrive drive;
    public final FriendlyIMU imu;
    public final LauncherWheelSimple launchWheelSimple;
    public final LauncherWheel launchWheel;
    public final LauncherGate launchGate;
    public final LauncherGateServo launchGateServo;
    public final MotorIntake motorIntake;
    public final RotaryIntake rotaryIntake;
    public final MotorIndexer motorIndexer;
    public final Indexer indexer;
    public final Loader loader;
    public final ArtifactSensor artifactSensor;
    public final MotifReader motifReader;


    public final VoltageSensor voltageSensor;

    /** Returns a new Robot25 Object containing an instance of each subsystem (or a
     * blank wherever a subsystem is not present).
     * {@param parameters} Get these from {@link RobotRoundhouse25}
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public Robot25(Parameters parameters, HardwareMap hardwareMap) {

        extendedParameters = parameters.extendedParameters;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // The order in which the Subsystems get created is important, because some of them depend on others!

        if (parameters.imuParameters.present) {
            imu = new FriendlyIMUImpl(parameters.imuParameters, hardwareMap);
        } else {
            imu = new FriendlyIMUBlank();
        }

        if (parameters.driveParameters.present) {
            parameters.driveParameters.imu = imu;
            parameters.driveParameters.batteryVoltageSensor = voltageSensor;
            drive = new RRMecanumDriveImpl(parameters.driveParameters);
        } else {
            drive = new RRMecanumDriveBlank();
        }


        if (parameters.launchWheelParams.present) {
            launchWheel = new LauncherWheelImpl(parameters.launchWheelParams);
        } else {
            launchWheel = new LauncherWheelBlank();
        }

        if (parameters.launchWheelSimpleParams.present) {
            launchWheelSimple = new LauncherWheelSimpleImpl(parameters.launchWheelSimpleParams);
        } else {
            launchWheelSimple = new LauncherWheelSimpleBlank();
        }

        if (parameters.launchGateParams.present) {
            launchGate = new LauncherGateImpl(parameters.launchGateParams);
        } else {
            launchGate = new LauncherGateBlank();
        }

        if (parameters.launcherGateServoParams.present) {
            launchGateServo = new LauncherGateServoImpl(parameters.launcherGateServoParams);
        } else {
            launchGateServo = new LauncherGateServoBlank();
        }

        if (parameters.motorIntakeParams.present) {
            motorIntake = new MotorIntakeImpl(parameters.motorIntakeParams);
        } else {
            motorIntake = new MotorIntakeBlank();
        }

        if (parameters.rotaryIntakeParams.present) {
            rotaryIntake = new RotaryIntakeImpl(parameters.rotaryIntakeParams);
        } else {
            rotaryIntake = new RotaryIntakeBlank();
        }

        if (parameters.indexerParameters.present) {
            indexer = new IndexerImpl(parameters.indexerParameters);
        } else {
            indexer = new IndexerBlank();
        }

        if (parameters.motorIndexerParameters.present) {
            motorIndexer = new MotorIndexerImpl(parameters.motorIndexerParameters);
        } else {
            motorIndexer = new MotorIndexerBlank();
        }

        if (parameters.loaderParameters.present) {
            loader = new LoaderImpl(parameters.loaderParameters);
        } else {
            loader = new LoaderBlank();
        }

        if (parameters.artifactSensorParameters.present) {
            artifactSensor = new ArtifactSensorImpl(parameters.artifactSensorParameters);
        } else {
            artifactSensor = new ArtifactSensorBlank();
        }

        if (parameters.motifReaderParameters.present) {
            motifReader = new MotifReaderImpl(parameters.motifReaderParameters);
        } else {
            motifReader = new MotifReaderBlank();
        }


        // Lynx stuff found in RR's SampleMecanumDrive
//        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /** Call this method in the loop portion of your OpMode.
     * <b>Remember this</b>; some subsystems will not function at all if you forget! */
    public void update() {
        drive.update();
        imu.update();

        launchWheelSimple.update();
        launchWheel.update();
        launchGate.update();
        motorIntake.update();
        rotaryIntake.update();
        motorIndexer.update();
        indexer.update();
        loader.update();
    }

}
