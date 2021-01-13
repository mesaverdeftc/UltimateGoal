package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class WobbleGoal {

    private Attachment wobbleArm = new Attachment();
    private Attachment wobbleServo = new Attachment();

    public void init(HardwareMap hardwareMap) {
        wobbleArm.init(hardwareMap, "wobble_arm_0", 0.0, 1.0);
        wobbleArm.up();

        wobbleServo.init(hardwareMap, "wobble_servo_1", 0.0, 1.0);
        wobbleServo.down();
    }

    public void setWobbleArm(boolean toggleState) {
        wobbleArm.toggle(toggleState);
    }
    public void setWobbleServo(boolean toggleState) {
        wobbleServo.toggle(toggleState);
    }
}