/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="Iterative Opmode")
public class Teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DriveTrain driveTrain = new DriveTrain();
    private Launcher launcher = new Launcher();
    private Attachment wobbleArm = new Attachment();
    private Attachment wobbleServo = new Attachment();
    private ButtonToggle buttonY = new ButtonToggle();
    private ButtonToggle buttonA = new ButtonToggle();
    private ButtonToggle buttonB = new ButtonToggle();
    private ButtonToggle button_rb = new ButtonToggle();
    private ButtonToggle button_lb = new ButtonToggle();
    private ButtonToggle button_dpad_down = new ButtonToggle();

    private ButtonToggle left_bumper = new ButtonToggle();
    private ButtonToggle right_bumper = new ButtonToggle();

    private ButtonToggle buttonX2 = new ButtonToggle();
    private ButtonToggle buttonB2 = new ButtonToggle();

    private boolean slowmode = false;
    private boolean fieldCentric = false;

    private DcMotor intakeMotor = null;

    double launcherSpeed = 0.55;
    boolean isLaunching = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        driveTrain.init(hardwareMap);
        if(!Constants.isStrafer) { // This means it is asking if it is the tileRunner. Look at the '!' in the statement
            launcher.init(hardwareMap);
            intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor_1");
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        wobbleArm.init(hardwareMap, "wobble_arm_0", 0.0, 1.0);
        wobbleServo.init(hardwareMap, "wobble_servo_1", 0.0, 1.0);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        if (!Constants.isStrafer) {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor_1");
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double left_x;
        double left_y;
        double right_x;


        if(Constants.isStrafer) {
            left_x = gamepad1.left_stick_x;
            left_y = -gamepad1.left_stick_y;
            right_x = gamepad1.right_stick_x;
        } else {
            left_x = gamepad1.left_stick_x;
            left_y = -gamepad1.left_stick_y;
            right_x = gamepad1.right_stick_x;
        }


        driveTrain.drive(left_x,left_y, right_x, fieldCentric, slowmode, telemetry);

//         Show the elapsed game time and wheel power.
        if(!Constants.isStrafer) {
            if(buttonA.toggled(gamepad1.a)) {
                intakeMotor.setPower(1.0);
            }
            else if(buttonB.toggled(gamepad1.b)) {
                intakeMotor.setPower(0.0);
            }
        }

        if(right_bumper.toggled(gamepad2.right_bumper)) {
            launcherSpeed+=0.01;
        }
        else if (left_bumper.toggled(gamepad2.left_bumper)) {
            launcherSpeed-=0.01;
        }

        if (buttonX2.toggled(gamepad2.x)) {
            if (buttonX2.toggleState) {
                isLaunching = true;
                launcher.run(launcherSpeed);
            }
            else
                launcher.stop();
        }

        if(isLaunching)
            launcher.run(launcherSpeed);

        if (buttonB2.toggled(gamepad2.b)) {
            launcher.launch(buttonB2.toggleState);
        }

//        if(buttonA.toggled(gamepad1.a)) {
//            wobbleArm.toggle(buttonA.toggleState);
//        }
//
//        if(buttonB.toggled(gamepad1.b)) {
//            wobbleServo.toggle(buttonB.toggleState);
//        }

        if (fieldCentric)
            telemetry.addData("Field Centric", "true");
        else
            telemetry.addData("Field Centric", "false");


        telemetry.addData("Button A Toggle State:", buttonA.toggleState);
        telemetry.addData("Encoders", "lf = %d, lr = %d, rf = %d, rr = %d ",
                driveTrain.leftFrontMotor.getCurrentPosition(),
                driveTrain.leftRearMotor.getCurrentPosition(),
                driveTrain.rightFrontMotor.getCurrentPosition(),
                driveTrain.rightRearMotor.getCurrentPosition());
        telemetry.addData("Values", "leftX = %.2f, leftY = %.2f", left_x, left_y);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftRear (%.2f), rightRear (%.2f)",
                driveTrain.leftFrontPower, driveTrain.rightFrontPower, driveTrain.leftRearPower, driveTrain.rightRearPower);
        telemetry.addData("Heading", "%.1f", driveTrain.getHeading());
        telemetry.addData("Launcher Speed", "%.2f", launcherSpeed);


    }

    @Override
    public void stop() {}
}