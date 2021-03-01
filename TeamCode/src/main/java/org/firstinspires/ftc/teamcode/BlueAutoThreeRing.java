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

@Autonomous(name="BlueAutoLeftThreeRing", group="Linear Opmode")
//@Disabled
public class BlueAutoThreeRing extends LinearOpMode{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();

//    private Attachment wobbleArm = new Attachment();
//    private Attachment wobbleServo = new Attachment();

    private Launcher launcher = new Launcher();

//    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
//    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
//
//    private static final int HORIZON = 135; // horizon value to tune
//
//    private static final boolean DEBUG = false; // if debug is wanted, change to true
//
//    private static final boolean USING_WEBCAM = false; // change to true if using webcam
//    private static final String WEBCAM_NAME = ""; // insert webcam name from configuration if using webcam

//    private bounceBaccPipeline pipeline;
//    private OpenCvCamera camera;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

//        initVision();

        driveTrain.init(hardwareMap);

//        wobbleArm.init(hardwareMap, "wobble_arm_0", 0.0, 1.0);
//        wobbleServo.init(hardwareMap, "wobble_servo_1", 0.0, 1.0);
        
//        wobbleServo.setServoForward();
//
//        wobbleArm.setPosition(0.65);
//        sleep(4000);
//        wobbleServo.setPosition(0.07);

        if(!Constants.isStrafer) {
            launcher.init(hardwareMap);
        }

        // make sure the imu gyro is calibrated before continuing.
        while (!opModeIsActive() && !driveTrain.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        launchRingInitialize(0.63);

        telemetry.addData("imu calib status", driveTrain.imu.getCalibrationStatus().toString());
        telemetry.update();

        //waiting for start
        telemetry.addData("Mode", "waiting for start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        movement();

        telemetry.addData("Path", "Complete");

        telemetry.addData("Encoders", "lf = %d, lr = %d, rf = %d, rr = %d ",
                driveTrain.leftFrontMotor.getCurrentPosition(),
                driveTrain.leftRearMotor.getCurrentPosition(),
                driveTrain.rightFrontMotor.getCurrentPosition(),
                driveTrain.rightRearMotor.getCurrentPosition());

        telemetry.update();
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
        launcher.stop();
    }

    private void movement() {
        sleep(500);
        driveTrain.gyroDrive_constant(this, runtime, 0.9, 60, 0, 15, telemetry);
        driveTrain.encoderStafe(this, runtime, 1, 24, false, 5);
        driveTrain.rotate(this, 0, 0.15);
        launchThreeRings();
        driveTrain.gyroDrive_constant(this, runtime, 0.2, 13, 0, 15, telemetry);
    }
}