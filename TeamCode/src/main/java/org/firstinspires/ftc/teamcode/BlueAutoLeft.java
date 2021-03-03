package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.EOCVtests.bounceBaccPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="BlueAutoLeft", group="Linear Opmode")
//@Disabled
public class BlueAutoLeft extends LinearOpMode{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();

    private Attachment wobbleArm = new Attachment();
    private Attachment wobbleServo = new Attachment();

    private Launcher launcher = new Launcher();

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 85; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = ""; // insert webcam name from configuration if using webcam

    private bounceBaccPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        initVision();

        driveTrain.init(hardwareMap);

        wobbleArm.init(hardwareMap, "wobble_arm_0", 0.0, 1.0);
        wobbleServo.init(hardwareMap, "wobble_servo_1", 0.0, 1.0);
        
        wobbleServo.setServoForward();

        if(!Constants.isStrafer) {
            wobbleArm.setPosition(0.45);
            sleep(4000);
            wobbleServo.setPosition(1);
        } else {
            wobbleArm.setPosition(0.65);
            sleep(4000);
            wobbleServo.setPosition(0);
        }

        if(!Constants.isStrafer) {
            launcher.init(hardwareMap);
            launchRingInitialize(0.617);
        }

        // make sure the imu gyro is calibrated before continuing.
        while (!opModeIsActive() && !driveTrain.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status", driveTrain.imu.getCalibrationStatus().toString());
        telemetry.update();

        //waiting for start
        telemetry.addData("Mode", "waiting for start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if(pipeline.getRectHeight() > 30) {
            camera.closeCameraDevice();

            telemetry.addData("Prediction:", "FOUR");
            telemetry.update();

            fourStackMovement();
        } else if(pipeline.getRectHeight() <= 30 && pipeline.getRectHeight() != 0) {
            camera.closeCameraDevice();

            telemetry.addData("Prediction:", "ONE");
            telemetry.update();

//            oneStackMovement();
        } else {
            camera.closeCameraDevice();

            telemetry.addData("Prediction:", "ZERO");
            telemetry.update();

//            zeroStackMovement();
        }


        telemetry.addData("Path", "Complete");

        telemetry.addData("Encoders", "lf = %d, lr = %d, rf = %d, rr = %d ",
                driveTrain.leftFrontMotor.getCurrentPosition(),
                driveTrain.leftRearMotor.getCurrentPosition(),
                driveTrain.rightFrontMotor.getCurrentPosition(),
                driveTrain.rightRearMotor.getCurrentPosition());

        telemetry.update();

        telemetry.addData("Launching", "Complete");
        telemetry.update();

    }


    private void initVision() {
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

        camera.setPipeline(pipeline = new bounceBaccPipeline(telemetry, DEBUG));

        bounceBaccPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        bounceBaccPipeline.Config.setHORIZON(HORIZON);

        double min_width = bounceBaccPipeline.Config.getMIN_WIDTH();

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT));

    }

    private void launchRingInitialize(double speed) {
        sleep(500);
        launcher.run(speed);
    }

    private void launchOnce() {
        sleep(500);
        launcher.launch(false);
        sleep(500);
        launcher.launch(true);
    }

    private void launchThreeRings() {
        for (int i = 0; i < 3; i++) {
            launchOnce();
        }
        launcher.stop();
    }

    private void launchTwoRings() {
        for (int i = 0; i < 2; i++) {
            launchOnce();
        }
        launcher.stop();
    }

    private void launchOneRing() {
        launchOnce();
    }

    private void zeroStackMovement() {
        sleep(500);
        driveTrain.gyroDrive_constant(this, runtime, 0.9, 73, 0, 15, telemetry);
        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(0);
        sleep(500);

        driveTrain.encoderStafe(this, runtime, 0.6, 40, false, 15);
        driveTrain.gyroDrive_constant(this, runtime, -0.5, -15, 0, 15, telemetry);
        sleep(1000);
        driveTrain.gyroDrive_constant(this, runtime, -0.5, -32, 0, 15, telemetry);
        driveTrain.encoderStafe(this, runtime, 0.4, 4, false, 15);
        wobbleArm.setPosition(0.65);
        sleep(1000);
        driveTrain.encoderStafe(this, runtime, 0.4, 5, true, 15);
        sleep(500);
        wobbleServo.setPosition(0.07);
        sleep(1000);

        driveTrain.gyroDrive_constant(this, runtime, 0.5, 51, 0, 15, telemetry, true);
        driveTrain.encoderStafe(this, runtime, 0.4, 28, true, 15);

        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(0);
        sleep(500);
    }

    private void oneStackMovement() {
        sleep(500);
        driveTrain.gyroDrive_constant(this, runtime, 0.9, 96, 0, 15, telemetry);

        driveTrain.encoderStafe(this, runtime, 0.6, 28, false, 15);

        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(0);
        sleep(500);

        driveTrain.encoderStafe(this, runtime, 0.6, 10, false, 15);
        driveTrain.gyroDrive_constant(this, runtime, -0.5, -38, 0, 15, telemetry);
        sleep(1000);
        driveTrain.gyroDrive_constant(this, runtime, -0.5, -33.3, 0, 15, telemetry);
        driveTrain.encoderStafe(this, runtime, 0.4, 4, false, 15);
        wobbleArm.setPosition(0.65);
        sleep(1000);
        driveTrain.encoderStafe(this, runtime, 0.4, 5, true, 15);
        sleep(500);
        wobbleServo.setPosition(0.07);
        sleep(1000);

        driveTrain.gyroDrive_constant(this, runtime, 0.5, 80, 0, 15, telemetry,true);
        driveTrain.encoderStafe(this, runtime, 0.4, 10, true, 15);

        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(0);
        sleep(500);

        driveTrain.gyroDrive_constant(this, runtime, -0.5, -29, 0, 15, telemetry);
    }

    private void fourStackMovement() {
        sleep(500);
        driveTrain.gyroDrive_constant(this, runtime, -0.9, -50, 0, 15, telemetry);
        driveTrain.rotate(this, -162, -0.75);

//        sleep(500);

        driveTrain.encoderStafe(this, runtime, 0.7, 20, false, 15);

        telemetry.addData("Current Angle:", driveTrain.getHeading());
        telemetry.update();

        launchOneRing();

        driveTrain.rotate(this, 170, -0.2);
        sleep(200);
        launchOneRing();

        driveTrain.rotate(this, 165, -0.2);
        sleep(500);
        launchOneRing();

        driveTrain.rotate(this, -180, -0.2);
        driveTrain.gyroDrive_constant(this, runtime, 0.7, 40, -180, 15, telemetry);

        if(!Constants.isStrafer) {
            wobbleServo.setPosition(0);
        }
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(0);
        sleep(500);

//        driveTrain.gyroDrive_constant(this, runtime, -0.5, -62, -179, 15, telemetry);
//        sleep(1000);
//        driveTrain.gyroDrive_constant(this, runtime, -0.5, -32, -179, 15, telemetry);
//        driveTrain.encoderStafe(this, runtime, 0.4, 4, false, 15);
//        wobbleArm.setPosition(0.65);
//        sleep(1000);
//        driveTrain.encoderStafe(this, runtime, 0.4, 5, true, 15);
//        sleep(500);
//        wobbleServo.setPosition(0.07);
//        sleep(1000);
//
//        driveTrain.gyroDrive_constant(this, runtime, 0.5, 94, 0, 15, telemetry, true);
//        driveTrain.encoderStafe(this, runtime, 0.4, 28, true, 15);
//
//        sleep(500);
//        wobbleServo.setPosition(1);
//        sleep(500);
//        wobbleArm.setPosition(0);
//        sleep(500);
//
//        driveTrain.gyroDrive_constant(this, runtime, -0.5, -47, 0, 15, telemetry);
    }
}