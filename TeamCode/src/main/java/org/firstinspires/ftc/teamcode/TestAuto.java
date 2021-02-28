package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestAuto", group="Linear Opmode")
//@Disabled
public class TestAuto extends LinearOpMode{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();

    private Attachment wobbleArm = new Attachment();
    private Attachment wobbleServo = new Attachment();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        driveTrain.init(hardwareMap);

        wobbleArm.init(hardwareMap, "wobble_arm_0", 0.0, 1.0);
        wobbleServo.init(hardwareMap, "wobble_servo_1", 0.0, 1.0);

        wobbleServo.setServoForward();

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

//        driveTrain.gyroStrafe(this, runtime, 0.6, 40, Constants.Direction.STRAFE_RIGHT, 0, 10);
//        sleep(5000);
//        driveTrain.gyroStrafe(this, runtime, 0.6, 40, Constants.Direction.STRAFE_LEFT, 0, 10);

        driveTrain.turnAndMove(this, runtime, driveTrain, 0.5, 20, Constants.Direction.STRAFE_RIGHT, 10, telemetry);

        telemetry.addData("Path", "Complete");

        telemetry.addData("Encoders", "lf = %d, lr = %d, rf = %d, rr = %d ",
                driveTrain.leftFrontMotor.getCurrentPosition(),
                driveTrain.leftRearMotor.getCurrentPosition(),
                driveTrain.rightFrontMotor.getCurrentPosition(),
                driveTrain.rightRearMotor.getCurrentPosition());

        telemetry.update();

        sleep(30000);


    }
}