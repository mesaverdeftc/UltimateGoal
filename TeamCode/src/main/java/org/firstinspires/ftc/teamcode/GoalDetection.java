package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.opencv.core.Rect;
import org.opencv.core.Point;

@TeleOp 
public class GoalDetection extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = ""; // insert webcam name from configuration if using webcam

    private UGBasicHighGoalPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGBasicHighGoalPipeline());

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        waitForStart();

        while (opModeIsActive()) {
            Rect blueRect = pipeline.getBlueRect();
            Rect redRect = pipeline.getRedRect();

            Rect redPowershot = pipeline.getRedPowershot();
            Rect bluePowershot = pipeline.getBluePowershot();

            // Rect (x, y, width, height)

            if(blueRect != null) {
                telemetry.addData("blueRect: ", blueRect.toString());
            }

            if(redRect != null) {
                telemetry.addData("redRect: ", redRect.toString());
            }

            if(bluePowershot != null) {
                telemetry.addData("bluePowershot: ", bluePowershot.toString());
            }

            if(redPowershot != null) {
                telemetry.addData("redPowershot: ", redPowershot.toString());
            }

            telemetry.addData("isRedVisible: ", pipeline.isRedVisible());
            telemetry.addData("isRedPowershotVisible: ", pipeline.isRedPowershotVisible());

            telemetry.addData("isBlueVisible: ", pipeline.isBlueVisible());
            telemetry.addData("isBluePowershotVisible: ", pipeline.isBluePowershotVisible());
            telemetry.update();
        }
    }
}