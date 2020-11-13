package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ParkAuto", group="Linear Opmode")
//@Disabled
public class ParkAuto extends LinearOpMode{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
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
        driveTrain.gyroDrive(this, runtime, 0.4, 22, 0, 5);
        driveTrain.encoderStafe(this, runtime, 0.4, 11, DriveTrain.STRAFE_LEFT, 9); //left
        driveTrain.gyroDrive(this, runtime, 0.25, 9, 0, 5);
        sleep(500);
        sleep(500);
        driveTrain.gyroDrive(this, runtime, -0.4, -29, 0, 5);
        driveTrain.rotate(this, 85, 0.4);
        driveTrain.gyroDrive(this, runtime, 0.4, 13, 90, 5);
        sleep(500);
        driveTrain.encoderStafe(this, runtime, .4, 18, DriveTrain.STRAFE_LEFT, 5);
        driveTrain.gyroDrive(this, runtime, -0.4, -40, 90, 5);

        telemetry.addData("Path", "Complete");
        telemetry.update();


    }
}