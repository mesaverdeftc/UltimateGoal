package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    private DcMotor wobble_goal = null;
    private Attachment wobble_goal_servo = new Attachment();

    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = ""; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    private int cameraMonitorViewId;
    private double min_width;

    private int prediction;

    @Override
    public void runOpMode() {

        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        sleep(1000);

        
        if(driveTrain.isTileRunner()) {
            // wobble_goal and wobble_goal_servo
            wobble_goal.setPower(1);
            wobble_goal_servo.setPosition(-1);
        }

        prediction = getPrediction();
        telemetry.addData("PREDICTION: ", prediction);
    }

    public void initalizeOpenCV() {
        cameraMonitorViewId = this
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

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        min_width = UGContourRingPipeline.Config.getMIN_WIDTH();

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));
    }

    public void initializeAttachment() {
        if(driveTrain.isTileRunner()) {
            wobble_goal = hardwareMap.get(DcMotor.class, "wobble_goal_0");
            wobble_goal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wobble_goal.setDirection(DcMotor.Direction.FORWARD);

            wobble_goal_servo.init(hardwareMap, "wobble_servo_0", 0, 1.0);
        }
    }

    public void initializeIMU() {
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !driveTrain.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status", driveTrain.imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    public void initialize() {
        // initialize drive-train
        driveTrain.init(hardwareMap);
        // initialize OpenCV
        initalizeOpenCV();
        // initialize attachments
        initializeAttachment();
        // initialize IMU
        initializeIMU();

        // waiting for start
        telemetry.addData("Mode", "waiting for start");
    }

    public int getPrediction() {
        String height = pipeline.getHeight().toString();
        telemetry.addData("[Ring Stack] >>", height);
        telemetry.addData("Min Width:", min_width);
        telemetry.update();

        camera.closeCameraDevice();

        if(height == "FOUR") {
            return 4;
        } else if(height == "ONE") {
            return 1;
        } else {
            return 0;
        }
    }
}