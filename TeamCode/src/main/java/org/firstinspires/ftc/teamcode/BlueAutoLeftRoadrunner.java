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
    private static final double REFLECTION_MODIFIER = -1;

    private bounceBaccPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Vector2d dropOff1 = Vector2dModified(9,48);
        Vector2d dropOff2 = Vector2dModified(36,24);
        Vector2d dropOff4 = Vector2dModified(63,48);
        Vector2d shootingSite = Vector2dModified(-5,8);
        Vector2d pickUpSite = Vector2dModified(-37,4);
        Vector2d pickUpSite2 = Vector2dModified(-37,9);
        Vector2d parkSite = Vector2dModified(0,-12);

        /* Trajectories */
        Trajectory zeroDropOff = drive.trajectoryBuilder(startPose)
                .lineTo(dropOff1) // runs forward 73 inches
                .build();
        Trajectory oneDropOff = drive.trajectoryBuilder(startPose)
                .lineTo(dropOff1)
                .splineTo(dropOff2, Math.toRadians(0))
                .build();
        Trajectory fourDropOff = drive.trajectoryBuilder(startPose)
                .lineTo(dropOff1)
                .splineTo(dropOff2, Math.toRadians(0))
                .build();
        Trajectory zeroShooting = drive.trajectoryBuilder(zeroDropOff.end())
                .lineTo(shootingSite) // moves right 40 inches and backwards 15 inches
                .addTemporalMarker(0, () -> wobbleServo.setPosition(1))
                .build();
        Trajectory oneShooting = drive.trajectoryBuilder(oneDropOff.end())
                .lineTo(shootingSite)
                .addTemporalMarker(0, () -> wobbleServo.setPosition(1))
                .build();
        Trajectory fourShooting = drive.trajectoryBuilder(fourDropOff.end())
                .lineTo(shootingSite)
                .addTemporalMarker(0, () -> wobbleServo.setPosition(1))
                .build();
        Trajectory pickUp = drive.trajectoryBuilder(zeroShooting.end())
                .lineTo(pickUpSite) // moves right 4 inches and backwards 32 inches
                .splineTo(pickUpSite2, Math.toRadians(0))
                .build();
        Trajectory finalDropOff = drive.trajectoryBuilder(pickUp.end())
                .lineTo(dropOff1)
                .build();
        Trajectory parking = drive.trajectoryBuilder(finalDropOff.end())
                .lineTo(parkSite)
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
            drive.followTrajectory(zeroDropOff);
            sleep(500);
            wobbleServo.setPosition(1);
            sleep(500);
            wobbleArm.setPosition(0);
            sleep(500);

            drive.followTrajectory(zeroShooting);
            sleep(1000);
            drive.followTrajectory(pickUp);
            wobbleArm.setPosition(0.65);
            sleep(1000);
            drive.followTrajectory(finalDropOff);
            sleep(500);
            wobbleServo.setPosition(0.07);
            sleep(1000);

            drive.followTrajectory(parking);

            sleep(500);
            wobbleServo.setPosition(1);
            sleep(500);
            wobbleArm.setPosition(0);
            sleep(500);
        } else if (pipeline.getRectHeight() <= 30 && pipeline.getRectHeight() != 0) {
            camera.closeCameraDevice();

            telemetry.addData("Prediction:", "ONE");
            telemetry.update();

            sleep(500);
            drive.followTrajectory(oneDropOff);
            sleep(500);
            wobbleServo.setPosition(1);
            sleep(500);
            wobbleArm.setPosition(0);
            sleep(500);

            drive.followTrajectory(oneShooting);
            sleep(1000);
            drive.followTrajectory(pickUp);
            wobbleArm.setPosition(0.65);
            sleep(1000);
            drive.followTrajectory(finalDropOff);
            sleep(500);
            wobbleServo.setPosition(0.07);
            sleep(1000);

            drive.followTrajectory(parking);

            sleep(500);
            wobbleServo.setPosition(1);
            sleep(500);
            wobbleArm.setPosition(0);
            sleep(500);
//            oneStackMovement();
        } else {
            camera.closeCameraDevice();

            telemetry.addData("Prediction:", "ZERO");
            telemetry.update();

            sleep(500);
            drive.followTrajectory(fourDropOff);
            sleep(500);
            wobbleServo.setPosition(1);
            sleep(500);
            wobbleArm.setPosition(0);
            sleep(500);

            drive.followTrajectory(fourShooting);
            sleep(1000);
            drive.followTrajectory(pickUp);
            wobbleArm.setPosition(0.65);
            sleep(1000);
            drive.followTrajectory(finalDropOff);
            sleep(500);
            wobbleServo.setPosition(0.07);
            sleep(1000);

            drive.followTrajectory(parking);

            sleep(500);
            wobbleServo.setPosition(1);
            sleep(500);
            wobbleArm.setPosition(0);
            sleep(500);

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
//        launcher.launch(false);
//        sleep(500);
//        launcher.launch(true);
//        sleep(500);
//        launcher.launch(false);
//        sleep(500);
//        launcher.launch(true);
//        sleep(500);
//        launcher.launch(false);
//        sleep(500);
//        launcher.launch(true);
        launcher.stop();
    }

    private Vector2d Vector2dModified(double x, double y) {
        return new Vector2d(x * VECTOR_MODIFIER, y * VECTOR_MODIFIER * REFLECTION_MODIFIER);
    }

    private double getDistanceFrom(double x, double y){
        Pose2d poseEstimate = drive.getPoseEstimate();
        return Math.sqrt(Math.pow(poseEstimate.getX() - x, 2) + Math.pow(poseEstimate.getY() - y, 2));
    }


//    private void zeroStackMovement() {
//        sleep(500);
//        drive.followTrajectory(zero1);
//        sleep(500);
//        wobbleServo.setPosition(1);
//        sleep(500);
//        wobbleArm.setPosition(0);
//        sleep(500);
//
//        drive.followTrajectory(zero2);
//        sleep(1000);
//        drive.followTrajectory(zero3);
//        wobbleArm.setPosition(0.65);
//        sleep(1000);
//        drive.followTrajectory(zero4);
//        sleep(500);
//        wobbleServo.setPosition(0.07);
//        sleep(1000);
//
//        drive.followTrajectory(zero5);
//
//        sleep(500);
//        wobbleServo.setPosition(1);
//        sleep(500);
//        wobbleArm.setPosition(0);
//        sleep(500);
//    }
//
//    private void oneStackMovement() {
//        sleep(500);
//        drive.gyroDrive_constant(this, runtime, 0.9, 96, 0, 15, telemetry);
//
//        drive.encoderStafe(this, runtime, 0.6, 28, false, 15);
//
//        sleep(500);
//        wobbleServo.setPosition(1);
//        sleep(500);
//        wobbleArm.setPosition(0);
//        sleep(500);
//
//        drive.encoderStafe(this, runtime, 0.6, 10, false, 15);
//        drive.gyroDrive_constant(this, runtime, -0.5, -38, 0, 15, telemetry);
//        sleep(1000);
//        drive.gyroDrive_constant(this, runtime, -0.5, -33.3, 0, 15, telemetry);
//        drive.encoderStafe(this, runtime, 0.4, 4, false, 15);
//        wobbleArm.setPosition(0.65);
//        sleep(1000);
//        drive.encoderStafe(this, runtime, 0.4, 5, true, 15);
//        sleep(500);
//        wobbleServo.setPosition(0.07);
//        sleep(1000);
//
//        drive.gyroDrive_constant(this, runtime, 0.5, 80, 0, 15, telemetry);
//        drive.encoderStafe(this, runtime, 0.4, 10, true, 15);
//
//        sleep(500);
//        wobbleServo.setPosition(1);
//        sleep(500);
//        wobbleArm.setPosition(0);
//        sleep(500);
//
//        drive.gyroDrive_constant(this, runtime, -0.5, -29, 0, 15, telemetry);
//    }
//
//    private void fourStackMovement() {
//        sleep(500);
//        drive.gyroDrive_constant(this, runtime, 0.9, 120, 0, 15, telemetry);
//        sleep(500);
//        wobbleServo.setPosition(1);
//        sleep(500);
//        wobbleArm.setPosition(0);
//        sleep(500);
//
//        drive.encoderStafe(this, runtime, 0.6, 40, false, 15);
//        drive.gyroDrive_constant(this, runtime, -0.5, -62, 0, 15, telemetry);
//        sleep(1000);
//        drive.gyroDrive_constant(this, runtime, -0.5, -32, 0, 15, telemetry);
//        drive.encoderStafe(this, runtime, 0.4, 4, false, 15);
//        wobbleArm.setPosition(0.65);
//        sleep(1000);
//        drive.encoderStafe(this, runtime, 0.4, 5, true, 15);
//        sleep(500);
//        wobbleServo.setPosition(0.07);
//        sleep(1000);
//
//        drive.gyroDrive_constant(this, runtime, 0.5, 94, 0, 15, telemetry);
//        drive.encoderStafe(this, runtime, 0.4, 28, true, 15);
//
//        sleep(500);
//        wobbleServo.setPosition(1);
//        sleep(500);
//        wobbleArm.setPosition(0);
//        sleep(500);
//
//        drive.gyroDrive_constant(this, runtime, -0.5, -47, 0, 15, telemetry);
//    }
}