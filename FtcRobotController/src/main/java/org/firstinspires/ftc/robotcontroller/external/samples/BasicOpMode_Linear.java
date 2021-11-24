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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorLeftUp = null;
    private DcMotor motorLeftDown = null;
    private DcMotor motorRightUp = null;
    private DcMotor motorRightDown = null;
    private DcMotor motorXrail = null;
    private DcMotor motorSpinner = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorLeftUp = hardwareMap.get(DcMotor.class, "left_motor_up");
        motorRightUp = hardwareMap.get(DcMotor.class, "right_motor_up");
        motorLeftDown = hardwareMap.get(DcMotor.class, "left_motor_down");
        motorRightDown = hardwareMap.get(DcMotor.class, "right_motor_down");
        motorXrail = hardwareMap.get(DcMotor.class, "xrail_motor");
        motorSpinner = hardwareMap.get(DcMotor.class, "spinner_motor");

        double motorSpeed = 0.1;

        motorXrail.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorLeftUp.setDirection(DcMotor.Direction.FORWARD);
        motorRightUp.setDirection(DcMotor.Direction.REVERSE);
        motorLeftDown.setDirection(DcMotor.Direction.FORWARD);
        motorRightDown.setDirection(DcMotor.Direction.REVERSE);
        motorXrail.setDirection(DcMotor.Direction.REVERSE);
        motorSpinner.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                motorSpeed = .2;
            }
            if (gamepad1.left_bumper) {
                motorSpeed = 0.1;
            }

            // Setup a variable for each drive wheel to save power level for telemetry
//            double leftPower;
//            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
//            double drive = -gamepad1.left_stick_y;
//            double turn  =  gamepad1.right_stick_x;
//            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            //moves mecanum wheels forward and backward
            motorLeftUp.setPower(motorSpeed * gamepad1.left_stick_y);
            motorRightUp.setPower(motorSpeed * gamepad1.left_stick_y);
            motorLeftDown.setPower(motorSpeed * gamepad1.left_stick_y);
            motorRightDown.setPower(motorSpeed * gamepad1.left_stick_y);
            motorXrail.setPower(motorSpeed * gamepad2.right_stick_y);
            motorSpinner.setPower(motorSpeed * Math.abs(gamepad2.left_stick_x));



            if (gamepad1.left_stick_x > 0.05 || gamepad1.left_stick_x < -0.05 ) {
                //moves mecanum wheels left and right
                motorLeftDown.setPower(motorSpeed * gamepad1.left_stick_x );
                motorRightDown.setPower(motorSpeed * gamepad1.left_stick_x * -1);
                motorLeftUp.setPower(motorSpeed * gamepad1.left_stick_x * -1);
                motorRightUp.setPower(motorSpeed * gamepad1.left_stick_x );
            }

            //turn robot clockwise
            if (gamepad1.right_stick_x !=0) {
                motorLeftDown.setPower(motorSpeed * gamepad1.right_stick_x * -1);
                motorRightDown.setPower(motorSpeed * gamepad1.right_stick_x );
                motorLeftUp.setPower(motorSpeed * gamepad1.right_stick_x * -1);
                motorRightUp.setPower(motorSpeed * gamepad1.right_stick_x );

            }

            // Send calculated power to wheels


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Motor Speed", motorSpeed);
            telemetry.addData("left_stick_y", "%.3f", gamepad1.left_stick_y);
            telemetry.addData("left_stick_x", "%.3f", gamepad1.left_stick_x);
            telemetry.addData("right_stick_x", "%.3f", gamepad1.right_stick_x);
            telemetry.addData("right_bumper", gamepad1.right_bumper);
            telemetry.addData("left_bumper", gamepad1.left_bumper);
            telemetry.addData("xrail_motor", gamepad2.right_stick_y);
            telemetry.addData("spinner_motor", gamepad2.left_stick_x);
            telemetry.update();
        }
    }
}
