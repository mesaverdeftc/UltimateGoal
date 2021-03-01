package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.EOCVtests.bounceBaccPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "BlueAutoLeftRoadrunner", group = "Linear Opmode")
@Disabled
public class BlueAutoLeftRoadrunner extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    private Attachment wobbleArm = new Attachment();
    private Attachment wobbleServo = new Attachment();

    private Launcher launcher = new Launcher();

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 135; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = ""; // insert webcam name from configuration if using webcam

    private static final double VECTOR_MODIFIER = 1;

    private bounceBaccPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Vector2d dropOff1 = Vector2dModified(9,48);
        Vector2d dropOff2 = Vector2dModified(36,24);
        Vector2d dropOff3 = Vector2dModified(10,10);
        Vector2d shootingSite = Vector2dModified(-5,8);
        Vector2d pickUp = Vector2dModified(-37,4);
        Vector2d pickUp2 = Vector2dModified(-37,9);

        /** Trajectory for the zeroStackMovement */
        Trajectory zeroDropOff = drive.trajectoryBuilder(startPose)
                .lineTo(dropOff1) // runs forward 73 inches
                .build();
        Trajectory oneDropOff = drive.trajectoryBuilder(startPose)
                .lineTo(dropOff1)
                .splineTo(dropOff2, Math.toRadians(0))
                .build();
        Trajectory zeroShooting = drive.trajectoryBuilder(zeroDropOff.end())
                .lineTo(shootingSite) // moves right 40 inches and backwards 15 inches
                .build();
        Trajectory oneShooting = drive.trajectoryBuilder(oneDropOff.end())
                .lineTo(shootingSite)
                .build();
        Trajectory zero3 = drive.trajectoryBuilder(zeroShooting.end())
                .lineTo(pickUp) // moves right 4 inches and backwards 32 inches
                .build();
        Trajectory zero4 = drive.trajectoryBuilder(zero3.end())
                .lineTo(pickUp2) // move left 5 inches
                .build();
        Trajectory zero5 = drive.trajectoryBuilder(zero4.end())
                .lineTo(Vector2dModified(14, 37)) // move left 28 inches and forward 51 inches
                .build();

        /* TODO: Trajectory for the oneStackMovement*/
        Trajectory one1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(33, 10)) // run forward 96 inches
                .lineTo(new Vector2d(10, 10)) // strafe right 28 inches
                .build();
        Trajectory one2 = drive.trajectoryBuilder(one1.end())
                .lineTo(new Vector2d(10, 10)) // strafe right 10 inches
                .splineTo(new Vector2d(10, 10), Math.toRadians(0)) // run backwards 38 inches
                .build();
        Trajectory one3 = drive.trajectoryBuilder(one2.end())
                .lineTo(new Vector2d(10, 10))  // run backwards 33.3 inches
                .splineTo(new Vector2d(10, 10), Math.toRadians(0)) // strafe right 4 inches
                .build();
        Trajectory one4 = drive.trajectoryBuilder(one3.end())
                .lineTo(new Vector2d(10, 10))  // strafe left 5 inches
                .build();
        Trajectory one5 = drive.trajectoryBuilder(one4.end())
                .lineTo(new Vector2d(10, 10)) // run forward 80 inches
                .splineTo(new Vector2d(10, 10), Math.toRadians(0)) // strafe left 10 inches
                .build();
        Trajectory one6 = drive.trajectoryBuilder(one5.end())
                .lineTo(new Vector2d(10, 10)) // run backwards 29 inches
                .build();

        /* TODO: Trajectory for the fourStackMovement*/
        Trajectory four1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(10, 10)) // run forwards 120 inches
                .build();
        Trajectory four2 = drive.trajectoryBuilder(four1.end())
                .lineTo(new Vector2d(10, 10)) // strafe forwards 120 inches
                .build();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        initVision();

        wobbleArm.init(hardwareMap, "wobble_arm_0", 0.0, 1.0);
        wobbleServo.init(hardwareMap, "wobble_servo_1", 0.0, 1.0);

        wobbleServo.setServoForward();

        wobbleArm.setPosition(0.65);
        sleep(4000);
        wobbleServo.setPosition(0.07);

        if (!Constants.isStrafer) {
            launcher.init(hardwareMap);
        }

        // make sure the imu gyro is calibrated before continuing.
        while (!opModeIsActive()) {
            sleep(50);
            idle();
        }

        //waiting for start
        telemetry.addData("Mode", "waiting for start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (pipeline.getRectHeight() > 30) {
            camera.closeCameraDevice();

            telemetry.addData("Prediction:", "FOUR");
            telemetry.update();

            sleep(500);
            drive.followTrajectory(zero1);
            sleep(500);
            wobbleServo.setPosition(1);
            sleep(500);
            wobbleArm.setPosition(0);
            sleep(500);

            drive.followTrajectory(zeroShooting);
            sleep(1000);
            drive.followTrajectory(zero3);
            wobbleArm.setPosition(0.65);
            sleep(1000);
            drive.followTrajectory(zero4);
            sleep(500);
            wobbleServo.setPosition(0.07);
            sleep(1000);

            drive.followTrajectory(zero5);

            sleep(500);
            wobbleServo.setPosition(1);
            sleep(500);
            wobbleArm.setPosition(0);
            sleep(500);
        } else if (pipeline.getRectHeight() <= 30 && pipeline.getRectHeight() != 0) {
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

    private void launchRings() {
        sleep(500);
        launcher.run(1.0);
        sleep(500);
        launcher.launch(false);
        sleep(500);
        launcher.launch(true);
        sleep(500);
        launcher.launch(false);
        sleep(500);
        launcher.launch(true);
        sleep(500);
        launcher.launch(false);
        sleep(500);
        launcher.launch(true);
        launcher.stop();
    }

    private Vector2d Vector2dModified(double x, double y) {
        return new Vector2d(x * VECTOR_MODIFIER, y * VECTOR_MODIFIER);
    }


    private void zeroStackMovement() {
        sleep(500);
        drive.followTrajectory(zero1);
        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(0);
        sleep(500);

        drive.followTrajectory(zero2);
        sleep(1000);
        drive.followTrajectory(zero3);
        wobbleArm.setPosition(0.65);
        sleep(1000);
        drive.followTrajectory(zero4);
        sleep(500);
        wobbleServo.setPosition(0.07);
        sleep(1000);

        drive.followTrajectory(zero5);

        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(0);
        sleep(500);
    }

    private void oneStackMovement() {
        sleep(500);
        drive.gyroDrive_constant(this, runtime, 0.9, 96, 0, 15, telemetry);

        drive.encoderStafe(this, runtime, 0.6, 28, false, 15);

        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(0);
        sleep(500);

        drive.encoderStafe(this, runtime, 0.6, 10, false, 15);
        drive.gyroDrive_constant(this, runtime, -0.5, -38, 0, 15, telemetry);
        sleep(1000);
        drive.gyroDrive_constant(this, runtime, -0.5, -33.3, 0, 15, telemetry);
        drive.encoderStafe(this, runtime, 0.4, 4, false, 15);
        wobbleArm.setPosition(0.65);
        sleep(1000);
        drive.encoderStafe(this, runtime, 0.4, 5, true, 15);
        sleep(500);
        wobbleServo.setPosition(0.07);
        sleep(1000);

        drive.gyroDrive_constant(this, runtime, 0.5, 80, 0, 15, telemetry);
        drive.encoderStafe(this, runtime, 0.4, 10, true, 15);

        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(0);
        sleep(500);

        drive.gyroDrive_constant(this, runtime, -0.5, -29, 0, 15, telemetry);
    }

    private void fourStackMovement() {
        sleep(500);
        drive.gyroDrive_constant(this, runtime, 0.9, 120, 0, 15, telemetry);
        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(0);
        sleep(500);

        drive.encoderStafe(this, runtime, 0.6, 40, false, 15);
        drive.gyroDrive_constant(this, runtime, -0.5, -62, 0, 15, telemetry);
        sleep(1000);
        drive.gyroDrive_constant(this, runtime, -0.5, -32, 0, 15, telemetry);
        drive.encoderStafe(this, runtime, 0.4, 4, false, 15);
        wobbleArm.setPosition(0.65);
        sleep(1000);
        drive.encoderStafe(this, runtime, 0.4, 5, true, 15);
        sleep(500);
        wobbleServo.setPosition(0.07);
        sleep(1000);

        drive.gyroDrive_constant(this, runtime, 0.5, 94, 0, 15, telemetry);
        drive.encoderStafe(this, runtime, 0.4, 28, true, 15);

        sleep(500);
        wobbleServo.setPosition(1);
        sleep(500);
        wobbleArm.setPosition(0);
        sleep(500);

        drive.gyroDrive_constant(this, runtime, -0.5, -47, 0, 15, telemetry);
    }
}