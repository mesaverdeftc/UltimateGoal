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
            launcher.run(0.8);
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

            fourStackMovement();

        } else if(pipeline.getRectHeight() <= 30 && pipeline.getRectHeight() != 0) {
            telemetry.addData("Prediction:", "ONE");
            telemetry.update();

            camera.closeCameraDevice();

            oneStackMovement();
        } else {
            telemetry.addData("Prediction:", "ZERO");
            telemetry.update();

            camera.closeCameraDevice();

            zeroStackMovement();
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

        driveTrain.gyroDrive_constant(this, runtime, -0.5, -12, 0, 15, telemetry);
        driveTrain.encoderStafe(this, runtime, 0.7, 26, false, 15);
        driveTrain.gyroDrive_constant(this, runtime, 0.5, 20, 0, 15, telemetry);

        telemetry.addData("Current Angle:", driveTrain.getHeading());
        telemetry.update();

        wobbleArm.setPosition(1);

        driveTrain.rotate(this, -0.4, -0.2);
        launcher.launchAutoZero(driveTrain, this, telemetry);

        driveTrain.gyroDrive_constant(this, runtime, -0.9, -53.4, -16.4, 15, telemetry);
        driveTrain.gyroDrive_constant(this, runtime, 0.9, 8, 0, 15, telemetry);
        driveTrain.rotate(this, -85, -0.4);

        driveTrain.gyroDrive_constant(this, runtime, 0.4, 4, -93, 15, telemetry);

        wobbleArm.setPosition(0);

        driveTrain.gyroDrive_constant(this, runtime, 0.4, 13.4, -93, 15, telemetry);

        sleep(500);

        wobbleServo.setPosition(0);

        sleep(500);

        driveTrain.rotate(this, 9.9, 0.4);
        driveTrain.gyroDrive_constant(this, runtime, 0.8, 50.3, 9.9, 15, telemetry);
        driveTrain.gyroDrive_constant(this, runtime, 0.35, 5, 9.9, 15, telemetry);

        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(1);
        sleep(500);

        driveTrain.encoderStafe(this, runtime, 0.3, 16, false, 15);

        driveTrain.gyroDrive_constant(this, runtime, 0.5, 8, 0, 10, telemetry);

        wobbleArm.setPosition(0);
        sleep(500);
    }

    private void oneStackMovement() {
        sleep(500);
        driveTrain.gyroDrive_constant(this, runtime, 0.9, 60.4, 0, 15, telemetry);

        driveTrain.encoderStafe(this, runtime, 0.7, 20, false, 15);

        driveTrain.rotate(this, 1, 0.15);

        driveTrain.gyroDrive_constant(this, runtime, -0.2, -3.4, 0, 15, telemetry);

        telemetry.addData("Current Angle:", driveTrain.getHeading());
        telemetry.update();
        
        wobbleArm.setPosition(1);

        launcher.launchAutoOne(driveTrain, this, telemetry);

        driveTrain.gyroDrive_constant(this, runtime, 0.8, 22, 0, 15, telemetry);

        wobbleArm.setPosition(0);
        sleep(500);
        wobbleServo.setPosition(1);

        driveTrain.gyroDrive_constant(this, runtime, -0.9, -84, -16.4, 15, telemetry);
        driveTrain.gyroDrive_constant(this, runtime, 0.9, 5.4, 0, 15, telemetry);
        driveTrain.rotate(this, -85, -0.4);

        driveTrain.gyroDrive_constant(this, runtime, 0.3, 18, -90, 15, telemetry);

        sleep(200);
        wobbleServo.setPosition(0);
        sleep(600);

        driveTrain.rotate(this, 6.5, 0.4);
        driveTrain.gyroDrive_constant(this, runtime, 0.8, 58, 6.5, 15, telemetry);

        driveTrain.encoderStafe(this, runtime, 0.5, 27.8, false, 15);
        driveTrain.gyroDrive_constant(this, runtime, 0.7, 19.4, 0, 10, telemetry);

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

        driveTrain.encoderStafe(this, runtime, 0.7, 22, false, 15);
        driveTrain.gyroDrive_constant(this, runtime, -0.8, -34, 0, 4, telemetry);

        driveTrain.rotate(this, 0.2, 0.1);

        telemetry.addData("Current Angle:", driveTrain.getHeading());
        telemetry.update();

        driveTrain.rotate(this, 1, 0.15);

        launcher.launchAutoFour(driveTrain, this, telemetry);

        wobbleArm.setPosition(0);
        sleep(500);

        driveTrain.encoderStafe(this, runtime, 0.7, 26, true, 15);

        driveTrain.gyroDrive_constant(this, runtime, -0.9, -61, 0, 15, telemetry);
        driveTrain.gyroDrive_constant(this, runtime, 0.9, 5.4, 0, 15, telemetry);
        driveTrain.rotate(this, -85, -0.4);

        driveTrain.gyroDrive_constant(this, runtime, 0.3, 13.4, -90, 15, telemetry);

        sleep(200);
        wobbleServo.setPosition(0);
        sleep(600);

        driveTrain.rotate(this, 7.2, 0.4);
        driveTrain.gyroDrive_constant(this, runtime, 0.8, 100, 7.2, 15, telemetry);


        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(1);
        sleep(500);

        driveTrain.gyroDrive_constant(this, runtime, -0.8, -32, 0, 10, telemetry);

        wobbleArm.setPosition(0);
        sleep(500);
    }
}