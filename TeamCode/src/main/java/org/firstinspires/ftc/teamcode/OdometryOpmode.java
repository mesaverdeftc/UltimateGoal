package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

@Autonomous(name = "OdometryOpmode", group="Linear Opmode")
public class OdometryOpmode extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain robot = new DriveTrain();

    // SET UP FOR THE WHEELS FOR ODOMETRY

    static final double CPR_ODOMETRY = 2048;
    // CPR = CYCLES per Revolution; NOT COUNTS per Revolution
    static final double DGR_ODOMETRY = 1.0;
    // DGR = Drive Gear Reduction
    static final double WHEEL_DIAMETER_INCHES = 1.96;
    static final double COUNTS_PER_INCH = (CPR_ODOMETRY * DGR_ODOMETRY) / (WHEEL_DIAMETER_INCHES * 3.1415);

    double left_dead_wheel = 0;
    double right_dead_wheel = 0;
    double bottom_dead_wheel = 0;

    double x_position = 0;
    double y_position = 0;
    // in inches

    public void runOpMode() {

        robot.init(hardwareMap);

        /* telemetry.addData("Status: Resetting Encoders");
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status: Successfully Reset Encoders");
        telemetry.update(); */

        waitForStart();

        // Set of coordinates the robot will go to
        GoToPosition(69, 69, 69, 69, 13);
        GoToPosition(420, 666, 69, 69, 13);

    }

    public void GoToPosition(double x_coordinate,
                             double y_coordinate,
                             double turn_speed, double drive_speed, double timeout) {

        double x_target;
        double y_target;
        double target_distance;
        double target_angle;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            x_target = x_coordinate - x_position;
            y_target = y_coordinate - y_position;

            target_distance = Math.sqrt(Math.pow(x_position, 2) + Math.pow(y_position, 2));

            target_angle = (Math.pow(target_distance, 2) + Math.pow(y_target, 2) - Math.pow(x_target, 2)) / Math.cos(2 * target_distance * y_target);

            runtime.reset();

            // Set angle correctly
            if (target_angle > (left_dead_wheel - right_dead_wheel)) {
                /* robot.leftDrive.setPower(turn_speed);
                robot.rightDrive.setPower(-turn_speed);
                while (((( left_dead_wheel - right_dead_wheel ) / 14 * Math.PI) * 360 < target_angle ) &&
                        opModeIsActive() && (runtime.seconds() < timeout) && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())){
                    */
                    sleep(1);

                } else {
                /* robot.leftDrive.setPower(-turn_speed);
                robot.rightDrive.setPower(turn_speed);
                while ((((left_dead_wheel - right_dead_wheel) / 14 * Math.PI) * 360 > target_angle ) &&
                        opModeIsActive() && (runtime.seconds() < timeout) && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())){
                    */
                    sleep(1);

                }


            /* robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0); */

            // Go to position
            /* robot.leftDrive.setPower(drive_speed);
            robot.rightDrive.setPower(drive_speed);
            while (left_dead_wheel == target_distance && right_dead_wheel == target_distance &&
                    opModeIsActive() && (runtime.seconds() < timeout) && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())){
                sleep(1);
            } */

            /* robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0); */

            //sleep(250);   // optional pause after each move
        }

    }


}