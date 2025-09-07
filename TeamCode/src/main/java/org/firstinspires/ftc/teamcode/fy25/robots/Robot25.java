package org.firstinspires.ftc.teamcode.fy25.robots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.framework.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.framework.subsystems.claw.ClawBlank;
import org.firstinspires.ftc.teamcode.framework.subsystems.claw.ClawImpl;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMU;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMUBlank;
import org.firstinspires.ftc.teamcode.framework.subsystems.friendlyimu.FriendlyIMUImpl;
import org.firstinspires.ftc.teamcode.framework.subsystems.pixelarm.ArmBlank;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntake;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntakeBlank;
import org.firstinspires.ftc.teamcode.framework.subsystems.rotaryintake.RotaryIntakeImpl;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDriveBlank;
import org.firstinspires.ftc.teamcode.framework.subsystems.rrmecanumdrive.RRMecanumDriveImpl;
import org.firstinspires.ftc.teamcode.framework.units.PIDConsts;
import org.firstinspires.ftc.teamcode.fy24.robots.RobotRoundhouse24;
import org.firstinspires.ftc.teamcode.fy24.subsystems.doublearm.DoubleArm;
import org.firstinspires.ftc.teamcode.fy24.subsystems.doublearm.DoubleArmImpl;

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
 * <code>Robot24 robot = new Robot24(RobotRoundhouse.getParamsAuto());</code>
 * <p>
 * See {@link RobotRoundhouse24} to find {@link Parameters} to pass in here. */
public class Robot25 {

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
        public Parameters(Claw.Parameters clawParameters, RotaryIntake.Parameters intakeParameters, FriendlyIMU.Parameters imuParameters, RRMecanumDrive.Parameters driveParameters, DoubleArm.Parameters doubleArmParameters,/*PixelArm.Parameters pixelArmParameters, PlaneLauncher.Parameters planeLauncherParameters,*/ ExtendedParameters extendedParameters) {
            this.clawParameters = clawParameters;
            this.intakeParameters = intakeParameters;
            this.imuParameters = imuParameters;
            this.driveParameters = driveParameters;
            this.doubleArmParameters = doubleArmParameters;
            this.extendedParameters = extendedParameters;
        }

        final Claw.Parameters clawParameters;
        final RotaryIntake.Parameters intakeParameters;
        final FriendlyIMU.Parameters imuParameters;
        final RRMecanumDrive.Parameters driveParameters;
        final DoubleArm.Parameters doubleArmParameters;
        final ExtendedParameters extendedParameters;
    }

    public final ExtendedParameters extendedParameters;

    public final Claw claw;
    public final RotaryIntake intake;
    public final FriendlyIMU imu;
    public final RRMecanumDrive drive;
    public final DoubleArm arm;


    public final VoltageSensor voltageSensor;

    /** Returns a new Robot24 containing an instance of each subsystem (or a
     * blank wherever a subsystem is not present).
     * {@param parameters} Robot24.Parameters
     * {@param hardwareMap} Pass in the hardwareMap provided by your OpMode. */
    public Robot25(Parameters parameters, HardwareMap hardwareMap) {

        extendedParameters = parameters.extendedParameters;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        if (parameters.clawParameters.present) {
            claw = new ClawImpl(parameters.clawParameters);
        } else {
            claw = new ClawBlank();
        }
        // above block and below statements work the same way
        intake = (parameters.intakeParameters.present) ? new RotaryIntakeImpl(parameters.intakeParameters): new RotaryIntakeBlank();
        imu = (parameters.imuParameters.present) ? new FriendlyIMUImpl(parameters.imuParameters, hardwareMap) : new FriendlyIMUBlank();

        parameters.driveParameters.imu = imu; // RRMecanumDrive needs an IMU, so we pass in the one we want here
                                              // (now that we've already created it - sequence is important!)
        parameters.driveParameters.batteryVoltageSensor = voltageSensor; // similar thing here

        drive = (parameters.driveParameters.present) ? new RRMecanumDriveImpl(parameters.driveParameters) : new RRMecanumDriveBlank();
        arm = (parameters.doubleArmParameters.present) ? new DoubleArmImpl(parameters.doubleArmParameters) : new ArmBlank();

        // Lynx stuff found in RR's SampleMecanumDrive
//        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /** Call this method in the loop portion of your OpMode.
     * Remember this; some subsystems will not function at all if you forget! */
    public void update() {
        claw.update();
        intake.update();
        imu.update();
        drive.update();
        arm.update();
    }

}
