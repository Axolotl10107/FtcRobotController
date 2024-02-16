/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.fy23.autoexperiment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */

// This actually uses AlliedDeterminationExample and prints the current analysis every time you
// press [A] (after Start is pressed).
@Disabled
@TeleOp
public class SuperAutonomousInitPrinter extends LinearOpMode
{
    OpenCvWebcam webcam;
    AlliedDeterminationExample.SkystoneDeterminationPipeline pipeline;
    AlliedDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis = AlliedDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default
    AlliedDeterminationExample.SkystoneDeterminationPipeline.SkystoneColor colorAnalysis = AlliedDeterminationExample.SkystoneDeterminationPipeline.SkystoneColor.BLUE; //default

    @Override
    public void runOpMode() {
        Telemetry.Log log = telemetry.log();
        try {
            realOpMode();
        } catch (Exception x) {
            log.add(x.getStackTrace().toString());
            while (opModeIsActive()) { sleep(100); }
        }
    }

    public void realOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new AlliedDeterminationExample.SkystoneDeterminationPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.addData("Realtime color", pipeline.getColor());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive())
        {
            analyzeImage();
            // Don't burn CPU cycles busy-looping in this sample
            telemetry.update(); //down here in case analyzeImage() takes too long
            sleep(50);
        }
    }

    void analyzeImage() {
        switch (snapshotAnalysis)
        {
            case LEFT:
            {
                telemetry.addData("Object Position", "Left");
                break;
            }

            case RIGHT:
            {
                telemetry.addData("Object Position", "Right");
                break;
            }

            case CENTER:
            {
                telemetry.addData("Object Position", "Center");
                break;
            }
        }

        switch (colorAnalysis) {
            case RED:
            {
                telemetry.addData("Object Color", "Red");
                break;
            }

            case BLUE:
            {
                telemetry.addData("Object Color", "Blue");
                break;
            }
        }
    }
}