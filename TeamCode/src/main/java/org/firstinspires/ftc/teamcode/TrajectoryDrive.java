package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class TrajectoryDrive {
    double leftFrontPower = 0.0;
    double rightFrontPower = 0.0;
    double leftRearPower = 0.0;
    double rightRearPower = 0.0;

    static final boolean DRIVE_FORWARD = true;
    static final boolean DRIVE_REVERSE = false;
    static final boolean STRAFE_LEFT = true;
    static final boolean STRAFE_RIGHT = false;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightRearDrive = null;

    private static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: ANDYMARK Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private DriveTrain driveTrain = new DriveTrain();

    private double angleOffset = 0;

    BNO055IMU imu;

    public void init(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRear");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Brake when power is set to zero (no coasting)
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double left_x, double left_y, boolean slowmode, double maxDriveSpeed, boolean lockedAngle, boolean lockedPosition) {
        if (slowmode) {
            left_y = left_y / 3.0;
            left_x = left_x / 3.0;
        }

        double left_hyptonuse = Math.sqrt(Math.pow(Math.abs(left_x), 2) + Math.pow(left_y, 2));
        double desiredAngle = (Math.pow(left_hyptonuse, 2) + Math.pow(Math.abs(left_y), 2) - Math.pow(Math.abs(left_x), 2)) / Math.cos(2 * left_hyptonuse * left_y);


        if (left_x < 0) {
            if (left_y > 0) {
                desiredAngle += 270;
            } else {
                desiredAngle += 180;
            }
        } else if (left_y < 0) {
            desiredAngle += 90;
        }

        double desiredAngleDifference = desiredAngle - driveTrain.getHeading();
        double turnSpeed = Math.abs(desiredAngleDifference) * (1 / maxDriveSpeed);

        if (desiredAngleDifference > 5 && desiredAngleDifference < -5 && lockedAngle == false && lockedPosition == false) {
            if (desiredAngleDifference > 0) {
                leftFrontDrive.setPower(turnSpeed);
                leftRearDrive.setPower(turnSpeed);
                rightFrontDrive.setPower(maxDriveSpeed);
                rightRearDrive.setPower(maxDriveSpeed);
            } else if (desiredAngleDifference < 0) {
                leftFrontDrive.setPower(maxDriveSpeed);
                leftRearDrive.setPower(maxDriveSpeed);
                rightFrontDrive.setPower(turnSpeed);
                rightRearDrive.setPower(turnSpeed);
            }
        } else {
            leftFrontDrive.setPower(maxDriveSpeed);
            leftRearDrive.setPower(maxDriveSpeed);
            rightFrontDrive.setPower(maxDriveSpeed);
            rightRearDrive.setPower(maxDriveSpeed);
        }

        if (lockedAngle == true && lockedPosition == false){

        }

        if (lockedPosition == true && lockedAngle == false){

        }

    }

    public double getHeading() {
        Orientation currentAngles;
        double heading;

        currentAngles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = -((AngleUnit.DEGREES.normalize(currentAngles.firstAngle))+ angleOffset + 360) %360;
        return heading;
    }
}
