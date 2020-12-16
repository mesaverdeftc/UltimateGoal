package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class MyOpmode extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory forward = drive.trajectoryBuilder(new Pose2d())
                .forward(36)

                .build();

        Trajectory backward = drive.trajectoryBuilder(new Pose2d())
                .back(36)
                .build();

        Trajectory strafeLeft = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(36)
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .build();



        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(forward);
        drive.followTrajectory(backward);
        drive.followTrajectory(strafeLeft);
    }
}