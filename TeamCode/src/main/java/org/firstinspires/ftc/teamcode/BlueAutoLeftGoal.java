package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.EOCVtests.bounceBaccPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="BlueAutoLeftGoal", group="Linear Opmode")
//@Disabled
public class BlueAutoLeftGoal extends LinearOpMode{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();

    private Attachment wobbleArm = new Attachment();
    private Attachment wobbleServo = new Attachment();

    private Launcher launcher = new Launcher();

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 135; // horizon value to tune

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

        wobbleServo.setPosition(0);

        if(!Constants.isStrafer) {
            launcher.init(hardwareMap);
            launcher.run(0.68);
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
            telemetry.addData("Prediction:", "FOUR");
            telemetry.update();

            camera.closeCameraDevice();

//            fourStackMovement();

        } else if(pipeline.getRectHeight() <= 30 && pipeline.getRectHeight() != 0) {
            telemetry.addData("Prediction:", "ONE");
            telemetry.update();

            camera.closeCameraDevice();

            oneStackMovement();
        } else {
            telemetry.addData("Prediction:", "ZERO");
            telemetry.update();

            camera.closeCameraDevice();

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

    private void zeroStackMovement() {
        sleep(500);
        driveTrain.gyroDrive_constant(this, runtime, 0.9, 57, 0, 15, telemetry);
//        driveTrain.rotate(this, -162, -0.75);

//        sleep(500);

        if(!Constants.isStrafer) {
            wobbleArm.setPosition(0);
            sleep(750);
            wobbleServo.setPosition(1);
        }

        driveTrain.gyroDrive_constant(this, runtime, -0.5, -8, 0, 15, telemetry);
        driveTrain.encoderStafe(this, runtime, 0.7, 20, false, 15);
        driveTrain.gyroDrive_constant(this, runtime, 0.5, 12, 0, 15, telemetry);

        telemetry.addData("Current Angle:", driveTrain.getHeading());
        telemetry.update();

        wobbleArm.setPosition(1);

        launcher.launchAutoZero(driveTrain, this, telemetry);

        driveTrain.gyroDrive_constant(this, runtime, -0.9, -35, -16.4, 15, telemetry);
        driveTrain.rotate(this, -85, -0.4);

        if(!Constants.isStrafer) {
            wobbleArm.setPosition(0);
        }

        driveTrain.gyroDrive_constant(this, runtime, 0.3, 11, -97, 15, telemetry);

        sleep(500);

        wobbleServo.setPosition(0);

        sleep(500);

        driveTrain.rotate(this, 6.5, 0.4);
        driveTrain.gyroDrive_constant(this, runtime, 0.8, 55.3, 6.5, 15, telemetry);

        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(1);
        sleep(500);

        driveTrain.encoderStafe(this, runtime, 0.3, 16, false, 15);
        driveTrain.gyroDrive_constant(this, runtime, 0.4, 13, 0, 10, telemetry);

        wobbleArm.setPosition(0);
        sleep(500);
    }

    private void oneStackMovement() {
        sleep(500);
        driveTrain.gyroDrive_constant(this, runtime, 0.9, 55, 0, 15, telemetry);

        driveTrain.encoderStafe(this, runtime, 0.7, 23.4, false, 15);
        driveTrain.gyroDrive_constant(this, runtime, -0.3, -10, 0, 4, telemetry);

        telemetry.addData("Current Angle:", driveTrain.getHeading());
        telemetry.update();

        launcher.launchAutoOne(driveTrain, this, telemetry);

        driveTrain.gyroDrive_constant(this, runtime, 0.8, 22, 0, 15, telemetry);

        wobbleArm.setPosition(0);
        sleep(500);
        wobbleServo.setPosition(1);

        driveTrain.gyroDrive_constant(this, runtime, -0.9, -58.4, -16.4, 15, telemetry);
        driveTrain.rotate(this, -85, -0.4);

        driveTrain.gyroDrive_constant(this, runtime, 0.3, 13, -95, 15, telemetry);

        sleep(200);
        wobbleServo.setPosition(0);
        sleep(600);

        driveTrain.rotate(this, 6.5, 0.4);
        driveTrain.gyroDrive_constant(this, runtime, 0.8, 56, 6.5, 15, telemetry);

        driveTrain.encoderStafe(this, runtime, 0.5, 24.2, false, 15);
        driveTrain.gyroDrive_constant(this, runtime, 0.7, 17.7, 0, 10, telemetry);

        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(1);
        sleep(500);

        driveTrain.gyroDrive_constant(this, runtime, -0.5, -10, 0, 10, telemetry);

        wobbleArm.setPosition(0);
        sleep(500);
    }

    private void fourStackMovement() {
        sleep(500);
        driveTrain.gyroDrive_constant(this, runtime, 0.9, 100, 0, 15, telemetry);

        wobbleArm.setPosition(0);
        sleep(500);
        wobbleServo.setPosition(1);
        sleep(400);
        wobbleArm.setPosition(1);
        sleep(200);

        driveTrain.encoderStafe(this, runtime, 0.7, 16, false, 15);
        driveTrain.gyroDrive_constant(this, runtime, -0.8, -37, 0, 4, telemetry);
        driveTrain.rotate(this, 0.6, 0.1);


        telemetry.addData("Current Angle:", driveTrain.getHeading());
        telemetry.update();

        sleep(900);
        launcher.launch(this, 1);

        launcher.run(0.65);
        sleep(900);
        launcher.launch(this, 1);

        driveTrain.rotate(this, -8, -0.2);

        launcher.run(0.65);
        sleep(900);
        launcher.launch(this, 1);

        wobbleArm.setPosition(0);
        sleep(500);

        driveTrain.gyroDrive_constant(this, runtime, -0.9, -40.8, -16.4, 15, telemetry);
        driveTrain.rotate(this, -85, -0.4);

        driveTrain.gyroDrive_constant(this, runtime, 0.3, 11.4, -95, 15, telemetry);

        sleep(200);
        wobbleServo.setPosition(0);
        sleep(600);

        driveTrain.rotate(this, 6.5, 0.4);
        driveTrain.gyroDrive_constant(this, runtime, 0.8, 98, 6.5, 15, telemetry);

        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(1);
        sleep(500);

        driveTrain.gyroDrive_constant(this, runtime, -0.8, -32, 0, 10, telemetry);

    }
}