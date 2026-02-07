package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

//    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
//            .rightFrontMotorName("rightFront")
//            .rightRearMotorName("rightBack")
//            .leftRearMotorName("leftBack")
//            .leftFrontMotorName("leftFront")
//            .leftFrontEncoderDirection(Encoder.REVERSE)
//            .leftRearEncoderDirection(Encoder.REVERSE)
//            .rightFrontEncoderDirection(Encoder.FORWARD)
//            .rightRearEncoderDirection(Encoder.FORWARD)
//            .forwardTicksToInches(-0.06654964911221173)
//            .strafeTicksToInches(0.3126641776356873)
//            .turnTicksToInches(0.009465250064493424)
//            .robotLength(8.75)
//            .robotWidth(13);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.5)
            .strafePodX(-8.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .customEncoderResolution(8192)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
//                .driveEncoderLocalizer(localizerConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

    }
}
