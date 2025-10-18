package org.firstinspires.ftc.teamcode.fy25.robots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMUBlank;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMUImpl;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDriveBlank;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDriveImpl;
import org.firstinspires.ftc.teamcode.framework.units.PIDConsts;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motorlntake.MotorIntake;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motorlntake.MotorIntakeBlank;
import org.firstinspires.ftc.teamcode.fy25.subsystems.motorlntake.MotorIntakeImpl;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launchergate.*;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel.LauncherWheel;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel.LauncherWheelBlank;
import org.firstinspires.ftc.teamcode.fy25.subsystems.launcherwheel.LauncherWheelImpl;

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

    /** Used when a subsystem that accepts multiple implementations of HardwareDevice receives one it doesn't expect */
    public static class InvalidDeviceClassException extends Exception {
        public InvalidDeviceClassException(String message) {
            super(message);
        }
        public InvalidDeviceClassException(String message, Throwable cause) {
            super(message, cause);
        }
    }

    /** Robot-specific parameters that are not used directly by the Robot but
     * by external things, usually processors. */
    public static class ExtendedParameters {
        /** Used by IMUCorrector.
         * We'll use this as our example. The Robot doesn't need this; only
         * IMUCorrector does. But it's Robot-specific, so it has to be a Robot
         * parameter. You can leave this un-set (or, rather, at the default
         * value of all 0s), but then IMUCorrector won't work with this Robot. */
        public PIDConsts hdgCorrectionPIDConsts = new PIDConsts(0, 0, 0, 0);
    }

    /** Subsystems are encapsulated in this class, so their Parameters are too. */
    public static class Parameters {
        public Parameters(
                ExtendedParameters extendedParameters,
                RRMecanumDrive.Parameters driveParameters,
                FriendlyIMU.Parameters imuParameters,

                LauncherWheel.Parameters launchWheelParams,
                LauncherGate.Parameters launchGateParams,
                MotorIntake.Parameters motorIntakeParams
        ) {
            // Every season
            this.extendedParameters = extendedParameters;
            this.driveParameters = driveParameters;
            this.imuParameters = imuParameters;

            // This season
            this.launchWheelParams = launchWheelParams;
            this.launchGateParams = launchGateParams;
            this.motorIntakeParams = motorIntakeParams;
        }

        final ExtendedParameters extendedParameters;
        final RRMecanumDrive.Parameters driveParameters;
        final FriendlyIMU.Parameters imuParameters;

        final LauncherWheel.Parameters launchWheelParams;
        final LauncherGate.Parameters launchGateParams;
        final MotorIntake.Parameters motorIntakeParams;
    }

    public final ExtendedParameters extendedParameters;

    public final RRMecanumDrive drive;
    public final FriendlyIMU imu;

    public final LauncherWheel launchWheel;
    public final LauncherGate launchGate;
    public final MotorIntake motorIntake;


    public final VoltageSensor voltageSensor;

    /** Returns a new Robot25 Object containing an instance of each subsystem (or a
     * blank wherever a subsystem is not present).
     * {@param parameters} Get these from {@link RobotRoundhouse25}
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public Robot25(Parameters parameters, HardwareMap hardwareMap) throws InvalidDeviceClassException {

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

        if (parameters.launchGateParams.present) {
            Class deviceClass = parameters.launchGateParams.deviceClass;
            if (deviceClass.equals(Servo.class)) {
                launchGate = new LauncherGateServoImpl(parameters.launchGateParams);
            } else if (deviceClass.equals(CRServo.class)) {
                launchGate = new LauncherGateCRServoImpl(parameters.launchGateParams);
            } else if (deviceClass.equals(DcMotorEx.class)) {
                launchGate = new LauncherGateMotorImpl(parameters.launchGateParams);
            } else {
                throw new InvalidDeviceClassException("launchGateParams.deviceClass must be 'Servo.class', 'CRServo.class', or 'DcMotorEx.class'.");
            }
        } else {
            launchGate = new LauncherGateBlank();
        }

        if (parameters.motorIntakeParams.present) {
            motorIntake = new MotorIntakeImpl(parameters.motorIntakeParams);
        } else {
            motorIntake = new MotorIntakeBlank();
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

        launchWheel.update();
        launchGate.update();
        motorIntake.update();
    }

}
