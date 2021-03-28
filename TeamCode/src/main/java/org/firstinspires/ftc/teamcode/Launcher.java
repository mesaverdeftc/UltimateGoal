package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {
    double launcherPower = 0.0;

    private DcMotor launcherMotor = null;

//    private DcMotorEx launcherMotor = null;
    public Attachment launcherServo = new Attachment();

    private boolean isFinishedLaunching = false;

    public void init(HardwareMap hardwareMap) {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        launcherMotor = hardwareMap.get(DcMotor.class, "launcher_motor_1");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setDirection(DcMotor.Direction.REVERSE);

        // Coast when power is set to zero
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherServo.init(hardwareMap, "launcher_servo_2", 0.0, 1.0);
//        launcherServo.up();
    }

    public void run(double power) {
        launcherPower = Range.clip(power, -1.0, 1.0);
        launcherMotor.setPower(power);

//        ((DcMotorEx) launcherMotor2).setVelocity(power);
    }

//    public double getVelocity() {
//        return launcherMotor.getVelocity();
//    }

    public void stop() {
        launcherMotor.setPower(0);
    }

    public void launch(LinearOpMode linearOpMode, int repetitions) {
        for(int i = 0; i != repetitions; i++) {
            launcherServo.toggle(false);
            linearOpMode.sleep(500);
            launcherServo.toggle(true);
        }
    }

    public void launchAutoZero(DriveTrain driveTrain, LinearOpMode linearOpMode, Telemetry telemetry) {

        launcherMotor.setPower(0.634);
        linearOpMode.sleep(900);
        launch(linearOpMode, 1);

//        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcherMotor.setPower(0.582);
        linearOpMode.sleep(800);
        launch(linearOpMode, 1);

        driveTrain.rotate(linearOpMode, -8.5, -0.2);

        launcherMotor.setPower(0.595);
        linearOpMode.sleep(800);
        launch(linearOpMode, 1);
    }

    public void launchAutoOne(DriveTrain driveTrain, LinearOpMode linearOpMode, Telemetry telemetry) {

        launcherMotor.setPower(0.633);
        linearOpMode.sleep(900);
        launch(linearOpMode, 1);

//        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcherMotor.setPower(0.582);
        linearOpMode.sleep(800);
        launch(linearOpMode, 1);

        driveTrain.rotate(linearOpMode, -8.8, -0.2);

        launcherMotor.setPower(0.594);
        linearOpMode.sleep(800);
        launch(linearOpMode, 1);
    }

    public void launchAutoFour(DriveTrain driveTrain, LinearOpMode linearOpMode, Telemetry telemetry) {

        launcherMotor.setPower(0.635);
        linearOpMode.sleep(900);
        launch(linearOpMode, 1);

//        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcherMotor.setPower(0.59);
        linearOpMode.sleep(800);
        launch(linearOpMode, 1);

        driveTrain.rotate(linearOpMode, -9.15, -0.2);

        launcherMotor.setPower(0.592);
        linearOpMode.sleep(800);
        launch(linearOpMode, 1);
    }

    public void getCurrentPosition(Telemetry telemetry, int repetitions) {
        for(int i = 0; i < repetitions; i++) {
            telemetry.addData("Launcher Motor Current Position: ", launcherMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    public double convertToTicks (double start) {
        return start / 28;
    }
}