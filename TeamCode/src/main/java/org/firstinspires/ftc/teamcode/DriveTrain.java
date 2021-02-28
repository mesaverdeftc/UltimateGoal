package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public DcMotor leftFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightRearMotor = null;

    private DcMotor tapeWinch = null;

    private boolean useTapeWinch = false;

    private static final double COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: ANDYMARK Motor Encoder
    private static final double MOTOR_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
//    private static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference

    private static final double WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference

    private static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * MOTOR_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    BNO055IMU imu;

    double globalAngle = 0;
    double lastAngles = 0;

    private double angleOffset = 0;

    public void init (HardwareMap hardwareMap) {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftRearMotor  = hardwareMap.get(DcMotor.class, "left_rear_motor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "right_rear_motor");

        // TODO: Run without encoders for teleop?
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(Constants.isStrafer) {
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        if (Constants.isStrafer) {
            leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
            rightRearMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        else {
            leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
            rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

            tapeWinch = hardwareMap.get(DcMotor.class, "tape_winch_0");
            tapeWinch.setDirection(DcMotor.Direction.FORWARD);
        }

        // Brake when power is set to zero (no coasting)
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void drive(double left_x, double left_y, double right_x, boolean fieldCentric, boolean slowmode, Telemetry telemetry) {

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

        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightRearMotor.setPower(rightRearPower);

        telemetry.addData("Connection Info: ", leftFrontMotor.getConnectionInfo());
        telemetry.addData("Odometry Value (LEFT_FRONT): ", leftFrontMotor.getCurrentPosition());
        telemetry.addData("Odometry Value (RIGHT_FRONT): ", rightFrontMotor.getCurrentPosition());
        telemetry.addData("Odometry Value (LEFT_REAR): ", leftRearMotor.getCurrentPosition());
    }

    public void stop() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
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
                                   double timeoutS,
                                   Telemetry telemetry,
                                   boolean... useTapeWinch) {

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

        if(inches > 0) {
            if (Math.abs(newSpeed) > 0.95) distanceThreshold = 32;
            else if (Math.abs(newSpeed) > 0.85) distanceThreshold = 29;
            else if (Math.abs(newSpeed) > 0.75) distanceThreshold = 27;
            else if (Math.abs(newSpeed) > 0.65) distanceThreshold = 22;
            else if (Math.abs(newSpeed) > 0.55) distanceThreshold = 20;
            else if (Math.abs(newSpeed) > 0.45) distanceThreshold = 14;
            else distanceThreshold = 10;
        } else {
            distanceThreshold = 10;
        }

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTargetPosition = leftFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            if(Constants.isStrafer) {
                leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {
                if (speed > 0) {
                    distanceRemaining = Range.clip(newTargetPosition - leftFrontMotor.getCurrentPosition(), 0, Integer.MAX_VALUE);
                } else {
                    distanceRemaining = Range.clip(leftFrontMotor.getCurrentPosition() - newTargetPosition, 0, Integer.MAX_VALUE);
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

                leftFrontMotor.setPower(motorSpeed.getLeftSpeed());
                rightFrontMotor.setPower(motorSpeed.getRightSpeed());
                leftRearMotor.setPower(motorSpeed.getLeftSpeed());
                rightRearMotor.setPower(motorSpeed.getRightSpeed());

                if(useTapeWinch[0] == true) {
                    tapeWinch.setPower(1);
                }

                telemetry.addData("left speed: ", motorSpeed.getLeftSpeed());
                telemetry.addData("right speed: ", motorSpeed.getRightSpeed());

                telemetry.addData("leftFrontMotor Position: ", leftFrontMotor.getCurrentPosition());
                telemetry.addData("distance remaining: ", distanceRemaining);

                telemetry.addData("newSpeed: ", newSpeed);
                telemetry.addData("distanceThreshold: ", distanceThreshold);
                telemetry.addData("newTargetPosition", newTargetPosition);

                telemetry.update();
            }

            stop();
        }
    }

    public void rotate(LinearOpMode linearOpMode, double desiredAngle, double speed) {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slowed down speed to be more accurate in angle turns
        if(speed > 0 ) {
            leftFrontMotor.setPower(-speed);
            rightFrontMotor.setPower(speed);
            leftRearMotor.setPower(-speed);
            rightRearMotor.setPower(speed);

        } else if(speed < 0);{
            leftFrontMotor.setPower(-speed);
            rightFrontMotor.setPower(speed);
            leftRearMotor.setPower(-speed);
            rightRearMotor.setPower(speed);
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
            newTargetPosition = leftFrontMotor.getCurrentPosition() + (int) (scale * inches * COUNTS_PER_INCH);

            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            leftFrontMotor.setPower(scale * speed);
            rightFrontMotor.setPower(scale * (-speed));
            leftRearMotor.setPower(scale * (-speed));
            rightRearMotor.setPower(scale * speed);

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {

                if (direction == STRAFE_RIGHT) {
                    if ((leftFrontMotor.getCurrentPosition()) > newTargetPosition) {
                        break;
                    }
                } else {
                    if ((leftFrontMotor.getCurrentPosition()) < newTargetPosition) {
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
                          double timeoutS,
                          Telemetry telemetry) {


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
            newTargetPosition = leftFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {
                if (speed > 0) {
                    // the problem
                    distanceRemaining = Range.clip(newTargetPosition - leftFrontMotor.getCurrentPosition(), 0, Integer.MAX_VALUE);
                } else {
                    distanceRemaining = Range.clip(leftFrontMotor.getCurrentPosition()- newTargetPosition, 0, Integer.MAX_VALUE);
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

                leftFrontMotor.setPower(motorSpeed.getLeftSpeed());
                rightFrontMotor.setPower(motorSpeed.getRightSpeed());
                leftRearMotor.setPower(motorSpeed.getLeftSpeed());
                rightRearMotor.setPower(motorSpeed.getRightSpeed());

                telemetry.addData("left speed: ", motorSpeed.getLeftSpeed());
                telemetry.addData("right speed: ", motorSpeed.getRightSpeed());

                telemetry.addData("leftFrontMotor Position: ", leftFrontMotor.getCurrentPosition());
                telemetry.addData("distance remaining: ", distanceRemaining);

                telemetry.addData("newSpeed: ", newSpeed);
                telemetry.addData("distanceThreshold: ", distanceThreshold);

                telemetry.update();
            }

            stop();
        }
    }

    public boolean isTileRunner() {
        if(leftFrontMotor.getConnectionInfo().toString() == "com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType@8c0db6e")  {
            return true;
        } else {
            return false;
        }
    }

}