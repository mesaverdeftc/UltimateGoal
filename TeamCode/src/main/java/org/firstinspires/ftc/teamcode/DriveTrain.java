package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrain {
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

    private static final double COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: ANDYMARK Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    BNO055IMU imu;

    double globalAngle = 0;
    double lastAngles = 0;

    private double angleOffset = 0;

    public void init (HardwareMap hardwareMap) {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");

        // TODO: Run without encoders for teleop?

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Brake when power is set to zero (no coasting)
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void drive(double left_x, double left_y, double right_x, boolean fieldCentric, boolean slowmode) {

        // Logrithmic controls as described at https://www.arthuradmiraal.nl/programming/ftc-taking-your-code-to-the-next-level/
        /*

            double x1, y1, x2;
            x1 = left_x * left_x * Math.signum(left_x);
            y1 = left_y * left_y * Math.signum(left_y);
            x2 = right_x * right_x * Math.signum(right_x);

            // Add a Dead Zone as described at https://www.arthuradmiraal.nl/programming/ftc-taking-your-code-to-the-next-level/
            if(Math.abs(x1) < 0.01) x1 = 0;
            if(Math.abs(y1) < 0.01) y1 = 0;
            if(Math.abs(x2) < 0.01) x2 = 0;

            left_x = x1;
            left_y = y1;
            right_x = x2;

        */

        // If our simulated manual transmission is in slowmode we divide the joystick values
        // by 3 to simulate a slower gear ratio on the robot.
        if(slowmode) {
            left_y = left_y / 3.0;
            left_x = left_x / 3.0;
            right_x = right_x / 3.0;
        }


        if (fieldCentric) {
            // Field centric driving using a rotation transform https://en.wikipedia.org/wiki/Rotation_matrix
            //double currentAngle = Math.toRadians(getHeading());
            //double new_x = left_x * Math.cos(currentAngle) - left_y * Math.sin(currentAngle);
            //double new_y = left_x * Math.sin(currentAngle) + left_y * Math.cos(currentAngle);

            //left_x = new_x;
            //left_y = new_y;

            left_x = -left_x;
            left_y = -left_y;

        }

        leftFrontPower   = Range.clip(left_y + right_x + left_x, -1.0, 1.0) ;
        rightFrontPower  = Range.clip(left_y - right_x - left_x, -1.0, 1.0) ;
        leftRearPower    = Range.clip(left_y + right_x - left_x, -1.0, 1.0) ;
        rightRearPower   = Range.clip(left_y - right_x + left_x, -1.0, 1.0) ;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
    }

    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
    }

    public void resetAngle() {
        Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleOffset = -((AngleUnit.DEGREES.normalize(currentAngles.firstAngle))+ 360) %360;
    }

    public double getHeading() {
        Orientation currentAngles;
        double heading;

        currentAngles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = -((AngleUnit.DEGREES.normalize(currentAngles.firstAngle))+ angleOffset + 360) %360;
        return heading;
    }

    public void gyroDrive_constant(LinearOpMode linearOpMode,
                                   ElapsedTime runtime,
                                   double speed,
                                   double inches,
                                   double angle,
                                   double timeoutS) {


        GyroSteerCorrection steerCorrection = new GyroSteerCorrection(imu, linearOpMode);
        int newTargetPosition;
        int distanceRemaining;
        int stopDistance = (int)(0.1 * COUNTS_PER_INCH);
        boolean direction;
        double newSpeed;
        int acceleration = 0;
        int deacceleration = 0;
        double distanceThreshold;

        if (inches > 0) {
            direction = DRIVE_FORWARD;
            newSpeed = Math.abs(speed);
        } else {
            direction = DRIVE_REVERSE;
            newSpeed = -Math.abs(speed);  // Set the speed negative if driving in reverse
        }

        if (Math.abs(newSpeed) > 0.95) distanceThreshold = 32;
        else if (Math.abs(newSpeed) > 0.85) distanceThreshold = 29;
        else if (Math.abs(newSpeed) > 0.75) distanceThreshold = 27;
        else if (Math.abs(newSpeed) > 0.65) distanceThreshold = 22;
        else if (Math.abs(newSpeed) > 0.55) distanceThreshold = 20;
        else if (Math.abs(newSpeed) > 0.45) distanceThreshold = 14;
        else distanceThreshold = 10;


        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTargetPosition = leftFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {
                if (speed > 0) {
                    distanceRemaining = Range.clip(newTargetPosition - leftFrontDrive.getCurrentPosition(), 0, Integer.MAX_VALUE);
                } else {
                    distanceRemaining = Range.clip(leftFrontDrive.getCurrentPosition()- newTargetPosition, 0, Integer.MAX_VALUE);
                }

                if (distanceRemaining < stopDistance) {
                    break;
                }
                //ramp the speed
                if(acceleration < 10) {
                    if(inches > 0) {
                        newSpeed = acceleration * .1;
                        newSpeed = Range.clip(newSpeed, 0, speed);
                    } else {
                        newSpeed = acceleration * -.1;
                        newSpeed = Range.clip(newSpeed, speed, 0);
                    }
                    acceleration++;
                }

                MotorSpeed motorSpeed = steerCorrection.correctMotorSpeed(newSpeed, angle);

                leftFrontDrive.setPower(motorSpeed.getLeftSpeed());
                rightFrontDrive.setPower(motorSpeed.getRightSpeed());
                leftRearDrive.setPower(motorSpeed.getLeftSpeed());
                rightRearDrive.setPower(motorSpeed.getRightSpeed());
            }

            stop();
        }
    }

    public void rotate(LinearOpMode linearOpMode, double desiredAngle, double speed) {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slowed down speed to be more accurate in angle turns
        if(speed > 0 ) {
            leftFrontDrive.setPower(-speed);
            rightFrontDrive.setPower(speed);
            leftRearDrive.setPower(-speed);
            rightRearDrive.setPower(speed);

        } else if(speed < 0);{
            leftFrontDrive.setPower(-speed);
            rightFrontDrive.setPower(speed);
            leftRearDrive.setPower(-speed);
            rightRearDrive.setPower(speed);
        }
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if(speed > 0) {
            while(!linearOpMode.isStopRequested() && desiredAngle >= angles.firstAngle) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                linearOpMode.telemetry.addData("Gyro","DesiredAngle: %.1f, Current Angle: %.1f", desiredAngle, AngleUnit.DEGREES.normalize(angles.firstAngle));
                linearOpMode.telemetry.update();
            }
        } else {
            while(!linearOpMode.isStopRequested() && desiredAngle <= angles.firstAngle) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                linearOpMode.telemetry.addData("Gyro", "DesiredAngle: %.1f, Current Angle: %.1f", desiredAngle, AngleUnit.DEGREES.normalize(angles.firstAngle));
                linearOpMode.telemetry.update();
            }
        }
        stop();
    }

    public void encoderStafe(LinearOpMode linearOpMode,
                             ElapsedTime runtime,
                             double speed,
                             double inches,
                             boolean direction,
                             double timeoutS) {
        int newTargetPosition;
        int scale;

        if (direction == STRAFE_LEFT)
            scale = -1;
        else
            scale = 1;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            // Determine new target position
            newTargetPosition = leftFrontDrive.getCurrentPosition() + (int) (scale * inches * COUNTS_PER_INCH);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            leftFrontDrive.setPower(scale * speed);
            rightFrontDrive.setPower(scale * (-speed));
            leftRearDrive.setPower(scale * (-speed));
            rightRearDrive.setPower(scale * speed);

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {

                if (direction == STRAFE_RIGHT) {
                    if ((leftFrontDrive.getCurrentPosition()) > newTargetPosition) {
                        break;
                    }
                } else {
                    if ((leftFrontDrive.getCurrentPosition()) < newTargetPosition) {
                        break;
                    }
                }
            }

            // Stop all motion;
            stop();
        }
    }



    public void gyroDrive(LinearOpMode linearOpMode,
                          ElapsedTime runtime,
                          double speed,
                          double inches,
                          double angle,
                          double timeoutS) {


        GyroSteerCorrection steerCorrection = new GyroSteerCorrection(imu, linearOpMode);
        int newTargetPosition;
        int distanceRemaining;
        int stopDistance = (int)(0.1 * COUNTS_PER_INCH);
        boolean direction;
        double newSpeed;
        int acceleration = 0;
        int deacceleration = 0;
        double distanceThreshold;

        if (inches > 0) {
            direction = DRIVE_FORWARD;
            newSpeed = Math.abs(speed);
        } else {
            direction = DRIVE_REVERSE;
            newSpeed = -Math.abs(speed);  // Set the speed negative if driving in reverse
        }

        if (Math.abs(newSpeed) > 0.95) distanceThreshold = 32;
        else if (Math.abs(newSpeed) > 0.85) distanceThreshold = 29;
        else if (Math.abs(newSpeed) > 0.75) distanceThreshold = 27;
        else if (Math.abs(newSpeed) > 0.65) distanceThreshold = 22;
        else if (Math.abs(newSpeed) > 0.55) distanceThreshold = 20;
        else if (Math.abs(newSpeed) > 0.45) distanceThreshold = 14;
        else distanceThreshold = 10;


        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTargetPosition = leftFrontDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {
                if (speed > 0) {
                    distanceRemaining = Range.clip(newTargetPosition - leftFrontDrive.getCurrentPosition(), 0, Integer.MAX_VALUE);
                } else {
                    distanceRemaining = Range.clip(leftFrontDrive.getCurrentPosition()- newTargetPosition, 0, Integer.MAX_VALUE);
                }

                if (distanceRemaining < stopDistance) {
                    break;
                }
                //ramp the speed
                if(acceleration < 10) {
                    if(inches > 0) {
                        newSpeed = acceleration * .1;
                        newSpeed = Range.clip(newSpeed, 0, speed);
                    } else {
                        newSpeed = acceleration * -.1;
                        newSpeed = Range.clip(newSpeed, speed, 0);
                    }
                    acceleration++;
                }

                if (distanceRemaining < (distanceThreshold * COUNTS_PER_INCH)) {
                    if (direction == DRIVE_FORWARD) {
                        newSpeed = speed - (deacceleration * 0.05);
                        newSpeed = Range.clip(newSpeed, 0.15, speed);
                    } else {
                        newSpeed = speed + (deacceleration * 0.05);
                        newSpeed = Range.clip(newSpeed, speed, -0.15);
                    }
                    deacceleration++;
                }

                MotorSpeed motorSpeed = steerCorrection.correctMotorSpeed(newSpeed, angle);

                leftFrontDrive.setPower(motorSpeed.getLeftSpeed());
                rightFrontDrive.setPower(motorSpeed.getRightSpeed());
                leftRearDrive.setPower(motorSpeed.getLeftSpeed());
                rightRearDrive.setPower(motorSpeed.getRightSpeed());
            }

            stop();
        }
    }

    public String getManufacturer() {
        return leftFrontDrive.getManufacturer().toString();
    }

}
