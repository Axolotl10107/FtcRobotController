package org.firstinspires.ftc.teamcode.fy23.teletest;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name="IMUinterfacePrinter", group="TeleTest")
public class IMUinterfacePrinter extends OpMode {

    IMU imu;

    RevHubOrientationOnRobot.LogoFacingDirection logoDir;
    RevHubOrientationOnRobot.LogoFacingDirection lastLogoDir;

    RevHubOrientationOnRobot.UsbFacingDirection usbDir;
    RevHubOrientationOnRobot.UsbFacingDirection lastUsbDir;

    AxesReference axesReference = AxesReference.INTRINSIC;
    AngleUnit angleUnit = AngleUnit.DEGREES;

    AxesOrder axesOrder = AxesOrder.ZYX;
    ArrayList<AxesOrder> axesOrderList = new ArrayList<>();

    Telemetry.Item revDirDisplay;
    Telemetry.Item usbDirDisplay;

    ElapsedTime buttonDelay;

    private void checkLogoDirControls() {
        if (gamepad1.dpad_up) { logoDir = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD; }
        if (gamepad1.dpad_down) { logoDir = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD; }
        if (gamepad1.dpad_left) { logoDir = RevHubOrientationOnRobot.LogoFacingDirection.LEFT; }
        if (gamepad1.dpad_right) { logoDir = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT; }
        if (gamepad1.right_bumper) { logoDir = RevHubOrientationOnRobot.LogoFacingDirection.UP; }
        if (gamepad1.left_bumper) { logoDir = RevHubOrientationOnRobot.LogoFacingDirection.DOWN; }
    }

    private void checkUsbDirControls() {
        if (gamepad1.y) { usbDir = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD; }
        if (gamepad1.a) { usbDir = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD; }
        if (gamepad1.x) { usbDir = RevHubOrientationOnRobot.UsbFacingDirection.LEFT; }
        if (gamepad1.b) { usbDir = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT; }
        if (gamepad1.right_trigger > 0.1) { usbDir = RevHubOrientationOnRobot.UsbFacingDirection.UP; }
        if (gamepad1.left_trigger > 0.1) { usbDir = RevHubOrientationOnRobot.UsbFacingDirection.DOWN; }
    }

    private void updateDirDisplays() {
        if (logoDir != null) {
            revDirDisplay.setValue(logoDir);
        }
        if (usbDir != null) {
            usbDirDisplay.setValue(usbDir);
        }
    }

    @Override
    public void init() {
        axesOrderList.addAll(Arrays.asList(AxesOrder.values()));

        imu = hardwareMap.get(IMU.class, "imu");

        buttonDelay = new ElapsedTime();

        telemetry.addLine("Please enter the Control Hub's orientation.");
        telemetry.addLine("REV Logo facing direction: Use D-Pad, or RB/LB for up/down.");
        telemetry.addLine("USB facing direction: Use face buttons, or RT/LT for up/down.");
        telemetry.addLine("(You can change these later while the OpMode is running.)");
        telemetry.addLine();
        revDirDisplay = telemetry.addData("REV Logo Direction", "unset").setRetained(true);
        usbDirDisplay = telemetry.addData("USB Logo Direction", "unset").setRetained(true);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        checkLogoDirControls();
        checkUsbDirControls();
        updateDirDisplays();
    }

    @Override
    public void loop() {
        init_loop(); // We always do this
        if (logoDir != null && usbDir != null) {
            mainMethod();
        } else {
            telemetry.addLine("You must set both the REV logo and USB port facing directions!");
            telemetry.update();
        }
    }

    private void mainMethod() {
        // check for and apply the user's changes to the IMU parameters
        if ( logoDir != lastLogoDir || usbDir != lastUsbDir ) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDir, usbDir)));
            lastLogoDir = logoDir;
            lastUsbDir = usbDir;
        }

        if (buttonDelay.milliseconds() > 300) {
            buttonDelay.reset();

            if (gamepad1.back) {
                if (axesReference == AxesReference.INTRINSIC) {
                    axesReference = AxesReference.EXTRINSIC;
                } else {
                    axesReference = AxesReference.INTRINSIC;
                }
            }

            if (gamepad1.left_stick_button) {
                if (angleUnit == AngleUnit.DEGREES) {
                    angleUnit = AngleUnit.RADIANS;
                } else {
                    angleUnit = AngleUnit.DEGREES;
                }
            }

            if (gamepad1.start) {
                int idx = axesOrderList.indexOf(axesOrder);
                if (idx == axesOrderList.size() - 1) {
                    idx = 0;
                } else {
                    idx += 1;
                }
                axesOrder = axesOrderList.get(idx);
            }
        }

        Orientation orientation = imu.getRobotOrientation(axesReference, axesOrder, angleUnit);

        telemetry.addData("Angle Unit (change with [left stick click]", angleUnit);
        telemetry.addData("Axes Reference (change with [back])", axesReference);
        telemetry.addData("Axes Order (change with [start]", axesOrder);
        telemetry.addData("1", orientation.firstAngle);
        telemetry.addData("2", orientation.secondAngle);
        telemetry.addData("3", orientation.thirdAngle);
        telemetry.update();
    }
}
