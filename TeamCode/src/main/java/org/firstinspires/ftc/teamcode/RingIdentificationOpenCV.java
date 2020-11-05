package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "RingIdentifcationOpenCV")
public class RingIdentificationOpenCV extends LinearOpMode {

    OpenCvInternalCamera phoneCam;
    SkystoneDetector detector = new SkystoneDetector();
    IdentifyStack.SkystoneDeterminationPipeline pipeline;

    private String position;

    @Override
    public void runOpMode() {
        // initialize hardware map here
        // gets id for CameraMonitor
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // passes the id from the CameraMonitor variable
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(detector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);


        waitForStart();

        while (opModeIsActive())
        {
            // code while robot is running

            telemetry.addData("Position", detector.position);
            telemetry.addData("Analysis", detector.analysis);
            telemetry.update();


            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }


    }
}
