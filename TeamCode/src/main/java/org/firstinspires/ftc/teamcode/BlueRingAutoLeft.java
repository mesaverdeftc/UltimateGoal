package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="BlueRingAutoLeft", group="Linear Opmode")
public class BlueRingAutoLeft extends LinearOpMode{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";

    private static final String VUFORIA_KEY =
            "AfyKdTL/////AAABmTmSsEgclk6kuOQfgCE8p5dzGFA7n2DfQGahPtf8ZXoso6frapO4YgRp1aO5rbQ/teMTZ7LofDIggTKwCFwPClwr3SJqslOL4Y0CKXeDFKZYD8Te9WKa+5rBQIxZfD+tiJkW+4HoBGTOSYooMiXYtF6uXEGKsM39HNmNLKq+xihLvuJ3h8kItp72xsRYXzdS2QowBDSz0ZgOuXK/KVBKls0KWqzadmKe3mqpnPBl13vcWArl9pbUEIXMhcSrUdSsbRCmLV+qoKgKpqdJGdGnzbQnK4PAkk2F2JFxVynJhBzY2VJpP/lxZkXGbxzcUw3fuI+fKMAYN7hye2xUGLd63mklCCms8bOcGBpyf9lFE1ae";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

//            tfod.setZoom(2.7, 1.78);
//            tfod.setZoom(1.78, 1.85);

            tfod.setClippingMargins(750,300,0,0);
        }

        driveTrain.init(hardwareMap);

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !driveTrain.imu.isGyroCalibrated())
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

        sleep(1000);

        telemetry.addData("START", "BEGINNING");
        telemetry.update();

        driveTrain.gyroDrive(this, runtime, 0.4, 2, 0, 5);

        sleep(1000);

        if(getPrediction() == 1) {
//            driveTrain.gyroDrive(this, runtime, 0.25, 5, 0, 5);
            telemetry.addData("Value: ", "1");
            telemetry.update();
        } else if(getPrediction() == 4) {
            driveTrain.gyroDrive(this, runtime, 0.25, 20, 0, 5);
            telemetry.addData("Value: ", "4");
            telemetry.update();
        }
        else if(getPrediction() == 10) {
            driveTrain.rotate(this, 360, 0.4);
            telemetry.addData("Value: ", "4");
            telemetry.update();
        } else {
            driveTrain.rotate(this, 85, 0.4);
            telemetry.addData("Value: ", "0");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();


    }

    public int getPrediction() {
        if (tfod != null) {
//            for(int i = 0; i < 500; i++) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for(Recognition recognition : updatedRecognitions) {
                    if(isSingleStackBasedOnHeight(recognition) == 1) {
                        return 1;
                    } else if(isSingleStackBasedOnHeight(recognition) == 4) {
                        return 4;
                    } else {
                        return 4;
                    }
                }
                return 10;
            } else {return 10;}
        } else {return 10;}
    }

    private int isSingleStackBasedOnHeight(Recognition recognition) {
        if(recognition.getHeight() < 100) {
            return 1;
        } else {
            return 4;
        }
    }

    /**\
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.48f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, "Four Stack", "Single Stack");
    }
}