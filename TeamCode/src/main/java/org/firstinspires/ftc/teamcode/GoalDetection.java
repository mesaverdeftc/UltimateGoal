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

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp 
public class GoalDetection extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = ""; // insert webcam name from configuration if using webcam

    private UGBasicHighGoalPipeline pipeline;
    private OpenCvCamera camera;

    public static final String VUFORIA_LICENSE_KEY = "AfyKdTL/////AAABmTmSsEgclk6kuOQfgCE8p5dzGFA7n2DfQGahPtf8ZXoso6frapO4YgRp1aO5rbQ/teMTZ7LofDIggTKwCFwPClwr3SJqslOL4Y0CKXeDFKZYD8Te9WKa+5rBQIxZfD+tiJkW+4HoBGTOSYooMiXYtF6uXEGKsM39HNmNLKq+xihLvuJ3h8kItp72xsRYXzdS2QowBDSz0ZgOuXK/KVBKls0KWqzadmKe3mqpnPBl13vcWArl9pbUEIXMhcSrUdSsbRCmLV+qoKgKpqdJGdGnzbQnK4PAkk2F2JFxVynJhBzY2VJpP/lxZkXGbxzcUw3fuI+fKMAYN7hye2xUGLd63mklCCms8bOcGBpyf9lFE1ae";

    @Override
    public void runOpMode() throws InterruptedException {

        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

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

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT));

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