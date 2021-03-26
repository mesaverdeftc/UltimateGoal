package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
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

public class TankDrive {

    double leftFrontPower;
    double rightFrontPower;
    double leftRearPower;
    double rightRearPower;

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


    BNO055IMU imu;
    double angleOffset = 0;


    public void init(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_motor");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_motor");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake when power is set to zero (no coasting)
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    enum controlMode {
        DEFAULT,
        LOCKED_POSITION,
        LOCKED_ANGLE,
        STATIONARY
    }

    public double joystickAngle(double left_x, double left_y) {

        //////////////////////////////////////////////////////////////////////////////
        //                  Converts the joystick position to angle                 //
        //////////////////////////////////////////////////////////////////////////////
        double joystickAngle = 0;
        double joystickRawAngle = Math.toDegrees(Math.atan((left_y) / (left_x)));

        if (left_x < 0) joystickAngle = 90 + joystickRawAngle;
        else if (left_x > 0) joystickAngle = 270 + joystickRawAngle;

        if (joystickAngle == 360) joystickAngle = 0;

        return joystickAngle;
    }

    public double offsetAngle(double left_x, double left_y) {

        double offsetAngle = 0;
        offsetAngle = getHeading() - joystickAngle(left_x, left_y);

        if (offsetAngle < 0) offsetAngle += 360;
        if (offsetAngle > 360 || offsetAngle == 360) offsetAngle -= 360;
        if (offsetAngle > 180 || offsetAngle == 180) offsetAngle -= 360;
        return offsetAngle;
    }

    public double getHeading() {
        Orientation currentAngles;
        double heading;

        currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        heading = ((AngleUnit.DEGREES.normalize(currentAngles.firstAngle)) + angleOffset + 360) % 360;
        return heading;
    }

    double joystickRadius;

    public void drive(double left_x,
                      double left_y,
                      boolean slowmode,
                      double maxDriveSpeed,
                      boolean lockedAngle,
                      boolean lockedPosition) {

        //////////////////////////////////////////////////////////////////////////////
        //                                  Slowmode                                //
        //////////////////////////////////////////////////////////////////////////////
        if (slowmode) {
            left_y = left_y / 3.0;
            left_x = left_x / 3.0;
        }

        joystickRadius = Math.sqrt(Math.pow(left_x, 2) + Math.pow(left_y, 2));

        //////////////////////////////////////////////////////////////////////////////
        //      Calculates the offset of the joystick angle to the robot heading    //
        //////////////////////////////////////////////////////////////////////////////
        double offsetAngle = offsetAngle(left_x, left_y);

        double leftPower = 0;
        double rightPower = 0;

        if (offsetAngle < 0) {
            leftPower = joystickRadius;
            rightPower = (joystickRadius - (Math.abs(offsetAngle) / 180) * 2) * joystickRadius * (maxDriveSpeed / 1);
        }
        if (offsetAngle > 0) {
            rightPower = joystickRadius;
            leftPower = (joystickRadius - (Math.abs(offsetAngle) / 180) * 2 * joystickRadius) * (maxDriveSpeed / 1);
        }
        if (offsetAngle > -5 && offsetAngle < 5) {
            leftPower = joystickRadius;
            rightPower = joystickRadius;
        }

        controlMode mode = controlMode.DEFAULT;

        if (lockedAngle && !lockedPosition) {
            mode = controlMode.LOCKED_ANGLE;
        }
        if(lockedPosition && !lockedAngle) {
            mode = controlMode.LOCKED_POSITION;
        }
        if(lockedPosition && lockedAngle){
            mode = controlMode.STATIONARY;
        }

        switch (mode) {
            case DEFAULT:

                leftFrontPower = leftPower;
                leftRearPower = leftPower;
                rightFrontPower = rightPower;
                rightRearPower = rightPower;

                if (joystickRadius < 0.25) {
                    leftFrontDrive.setPower(0);
                    leftRearDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightRearDrive.setPower(0);
                } else {
                    leftFrontDrive.setPower(leftFrontPower);
                    leftRearDrive.setPower(leftRearPower);
                    rightFrontDrive.setPower(rightFrontPower);
                    rightRearDrive.setPower(rightRearPower);
                }
                break;

            case LOCKED_ANGLE:
                //////////////////////////////////////////////////////////////////////////////
                //  When the angle is locked, the robot is always facing the same direction //
                //////////////////////////////////////////////////////////////////////////////

                left_x *= 100;
                left_y *= 100;

                double V = (100 - Math.abs(left_x)) * (left_y / 100) + left_y;
                double W = (100 - Math.abs(left_y)) * (left_x / 100) + left_x;

                rightFrontPower = ((V + W) / 2) * (maxDriveSpeed / 100);
                leftFrontPower = ((V - W) / 2) * (maxDriveSpeed / 100);
                rightRearPower = leftFrontPower;
                leftRearPower = rightFrontPower;

                if (joystickRadius < 0.25) {
                    leftFrontDrive.setPower(0);
                    leftRearDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightRearDrive.setPower(0);
                } else {
                    leftFrontDrive.setPower(leftFrontPower);
                    leftRearDrive.setPower(leftRearPower);
                    rightFrontDrive.setPower(rightFrontPower);
                    rightRearDrive.setPower(rightRearPower);
                }

                break;

            case LOCKED_POSITION:
                //////////////////////////////////////////////////////////////////////////////
                //     When the position is locked, the robot remains in the same place     //
                //////////////////////////////////////////////////////////////////////////////

                leftFrontPower = offsetAngle * -(maxDriveSpeed / 90);
                leftRearPower = leftFrontPower;
                rightFrontPower = -leftFrontPower;
                rightRearPower = rightFrontPower;

                if (joystickRadius < 0.25) {
                    leftFrontDrive.setPower(0);
                    leftRearDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightRearDrive.setPower(0);
                } else {
                    leftFrontDrive.setPower(leftFrontPower);
                    leftRearDrive.setPower(leftRearPower);
                    rightFrontDrive.setPower(rightFrontPower);
                    rightRearDrive.setPower(rightRearPower);
                }

                break;

        }
    }

}
