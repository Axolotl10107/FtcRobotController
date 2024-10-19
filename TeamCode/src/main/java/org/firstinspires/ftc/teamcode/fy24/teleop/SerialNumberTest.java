package org.firstinspires.ftc.teamcode.fy24.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.EmbeddedControlHubModule;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.internal.network.ControlHubDeviceNameManager;
import org.firstinspires.ftc.robotcore.internal.usb.LynxModuleSerialNumber;

@TeleOp(name="~Serial Number Test", group="")
public class SerialNumberTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("name: ",ControlHubDeviceNameManager.getControlHubDeviceNameManager().getDeviceName());
        telemetry.update();
        waitForStart();
    }
}
