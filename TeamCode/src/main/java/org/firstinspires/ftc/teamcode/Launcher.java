package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Launcher {
    double launcherPower = 0.0;

    private DcMotor launcherMotor = null;
    private Attachment launcherServo = new Attachment();

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
        launcherServo.up();
    }

    public void run(double power) {

        launcherPower = Range.clip(power, -1.0, 1.0);
        launcherMotor.setPower(launcherPower);
    }

    public void stop() {
        launcherMotor.setPower(0);
    }

    public void launch(boolean toggleState) {
        launcherServo.toggle(toggleState);
    }
}