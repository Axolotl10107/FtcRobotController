package org.firstinspires.ftc.teamcode.fy23.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.*;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.*;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.blank.hardwaredevice.DoubleArmBlank;
import org.firstinspires.ftc.teamcode.fy23.robot.subsystems.normalimpl.*;
import org.firstinspires.ftc.teamcode.fy23.units.PIDConsts;

/** Encapsulates all of the components that make up a robot. A simple way to centralize initialization in one place
 * (to avoid duplicated code across OpModes) and make it easier to work with multiple different robots, each of which is
 * defined by a {@link Parameters} class. The Parameters also contain calibration values that tune certain fine
 * behaviors to each robot. An OpMode no longer needs to set up every individual motor and servo. It needs only to
 * create a robot and use the powerful and convenient methods provided by its subsystems. */
public class Robot24 {

    /** Extra stuff that external stuff (like IMUCorrector) uses*/
    public static class ExtendedParameters {
        /** used by IMUCorrector */
        public PIDConsts hdgCorrectionPIDConsts;
    }

    public static class Parameters {
        public Parameters(Claw.Parameters clawParameters, RotaryIntake.Parameters intakeParameters, FriendlyIMU.Parameters imuParameters, RRMecanumDrive.Parameters driveParameters, DoubleArm.Parameters doubleArmParameters,/*PixelArm.Parameters pixelArmParameters, PlaneLauncher.Parameters planeLauncherParameters,*/ ExtendedParameters extendedParameters) {
            this.clawParameters = clawParameters;
            this.intakeParameters = intakeParameters;
            this.imuParameters = imuParameters;
            this.driveParameters = driveParameters;
            this.doubleArmParameters = doubleArmParameters;
            this.extendedParameters = extendedParameters;
        }

//        @Deprecated
//        double tpr; /** ticks per rotation */
//        @Deprecated
//        double wheelDiameter;
//        @Deprecated
//        double maxForwardSpeed;
//        double driveToStrafeDistCV; // conversion factor from driving distance to equivalent
        // strafing distance, in encoder ticks

        final Claw.Parameters clawParameters;
        final RotaryIntake.Parameters intakeParameters;
        final FriendlyIMU.Parameters imuParameters;
        final RRMecanumDrive.Parameters driveParameters;
        final DoubleArm.Parameters doubleArmParameters;
        final ExtendedParameters extendedParameters;
    }

//    @Deprecated
//    public final double TPR;
//    @Deprecated
//    public final double wheelDiameter;
//    @Deprecated
//    public final double wheelCircumference;
//    @Deprecated
//    public final double maxForwardSpeed;
    public final ExtendedParameters extendedParameters;

    public final Claw claw;
    public final RotaryIntake intake;
    public final FriendlyIMU imu;
    public final RRMecanumDrive drive;
    public final DoubleArm arm;


    public final VoltageSensor voltageSensor;

    /** Pass in an ElapsedTime to be used by subsystems. Useful for dependency injection. The other constructor creates
     * a normal ElapsedTime. */
    public Robot24(Parameters parameters, HardwareMap hardwareMap) {
//        TPR = parameters.tpr;
//        wheelDiameter = parameters.wheelDiameter;
//        wheelCircumference = Math.PI * wheelDiameter;
//        maxForwardSpeed = parameters.maxForwardSpeed;
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
        parameters.driveParameters.batteryVoltageSensor = voltageSensor; // similar thing here
        drive = (parameters.driveParameters.present) ? new RRMecanumDriveImpl(parameters.driveParameters) : (RRMecanumDrive) new MecanumDriveBlank();
        arm = (parameters.doubleArmParameters.present) ? new DoubleArmImpl(parameters.doubleArmParameters) : new DoubleArmBlank();
        // Lynx stuff found in RR's SampleMecanumDrive
//        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /** Call this method in the loop portion of your OpMode. */
    public void update() {
        claw.update();
        intake.update();
        imu.update();
        drive.update();
        arm.update();
    }

}
