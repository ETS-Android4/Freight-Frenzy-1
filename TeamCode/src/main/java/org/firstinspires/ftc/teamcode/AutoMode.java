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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


import java.util.List;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
enum rotate_dir {
    ROTATE_FORWARD_RIGHT,
    ROTATE_FORWARD_LEFT,
    ROTATE_BACKWARD_RIGHT,
    ROTATE_BACKWARD_LEFT,
    ROTATE_NONE
}

@Autonomous(name = "AutoMode", group = "Original")
public class AutoMode extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            "AbLCOaz/////AAABmR2DgtefskiUrRY30djF+uECtyuQc2HSw9leOaPJwjtmYlbxf2KLy1gvTwjdaE2Ce0hoCO97GjrvMSdErzPBkWCOmrxNUxjB+5EJa0UzUcd5A7rsohIstWvQfBtojVQsGh+ykbTTCuRCugDUw9HyVcJO1s+AdaZnzQlqefgZz+531xPRIZAxrOxbGSLFp5TWtECnM13ERkMpJNxNWBS+SUxkAXyZj+cAKaelgEDUNvR12VMdRuy7um5EmhzCP8qP94gVmoV8ghWleypt9NxY05p5jxFgCTyh54GXyyWMuXSjSIIEHZkxab3k1G/U1QDLnlEbTeD0aIsq1ETrkRzXtqiGA2FAj7K4tsZeoZA4yhzc";
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorLeftDriveUp = null;
    private DcMotor motorLeftDriveDown = null;
    private DcMotor motorRightDriveUp = null;
    private DcMotor motorRightDriveDown = null;
    private DcMotor motorXrailDrive = null;
    private DcMotor motorSpinnerDrive = null;
    private Servo servoTilterDrive = null;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Servo claw = null;
    private WebcamName webcam = null;


    @Override
    public void runOpMode() {


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initVuforia(hardwareMap);
        initTfod(hardwareMap);

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(3, 16.0 / 9.0);
        }
        telemetry.addData("Status", "Null");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorLeftDriveUp = hardwareMap.get(DcMotor.class, "left_motor_up");
        motorRightDriveUp = hardwareMap.get(DcMotor.class, "right_motor_up");
        motorLeftDriveDown = hardwareMap.get(DcMotor.class, "left_motor_down");
        motorRightDriveDown = hardwareMap.get(DcMotor.class, "right_motor_down");
        motorXrailDrive = hardwareMap.get(DcMotor.class, "xrail_motor");
        motorSpinnerDrive = hardwareMap.get(DcMotor.class, "spinner_motor");
        servoTilterDrive = hardwareMap.get(Servo.class, "tilter_servo");

        double motorSpeed = 1;
        double xrailSpeed = .5;
        //double position = servoTilterDrive.getPosition();
        double increment = 0.01;
        double Max_pos = .3;
        double Min_pos = 0.0;
        int count = 0;


        motorXrailDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSpinnerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorLeftDriveUp.setDirection(DcMotor.Direction.FORWARD);
        motorRightDriveUp.setDirection(DcMotor.Direction.REVERSE);
        motorLeftDriveDown.setDirection(DcMotor.Direction.FORWARD);
        motorRightDriveDown.setDirection(DcMotor.Direction.REVERSE);
        motorXrailDrive.setDirection(DcMotor.Direction.REVERSE);
        motorSpinnerDrive.setDirection(DcMotor.Direction.FORWARD);
        servoTilterDrive.setDirection(Servo.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        move_forward(4);

        // drive to box from starting position
        // check for cube
        //boolean foundDuck = checkForDuck();

        //move_forward(16);
        //sleep(1000);
/*
        if (checkForDuck()) {
            path1();
        }
        else{
           strafe_right(8);
           sleep(1000);
           if(checkForDuck()){
               path2();
           }
           else{
           sleep(1000);
               strafe_right(8);
              path3();
               }

           }

 */

       /*strafe_right(2);
        if (checkForDuck()) {
            path4();
        }
        else{
            strafe_left(8);
            sleep(1000);
            if(checkForDuck()){
                path5();
            }
            else{
                sleep(1000);
                strafe_left(8);
                path6();



            }



        }
*/     // make_a_turn(rotate_dir.ROTATE_FORWARD_RIGHT, 4);
        //sleep(3000);
        //if(checkForDuck()) {
          //  move_forward(3);
        //}
        //left side detection code will comment out if neccesary, right side for red
        if(checkForDuck()) {
            path2();
        }
        else{
            move_forward(2);
            make_a_turn(rotate_dir.ROTATE_FORWARD_LEFT, 4);
            sleep(1000);
            if(checkForDuck()){
                path1();
            }
            else{
                make_a_turn(rotate_dir.ROTATE_FORWARD_RIGHT, 11);
                sleep(1000);
                path3();
            }
            //right side duck detection code will comment out if neccesary, left side for red
            if(checkForDuck()) {
                path5();
            }
            else {
                move_forward(2);
                make_a_turn(rotate_dir.ROTATE_FORWARD_RIGHT, 4);
                sleep(1000);
                if(checkForDuck()) {
                    path4();
                }
                else {
                    make_a_turn(rotate_dir.ROTATE_FORWARD_LEFT, 11);
                    sleep(1000);
                    path3();
                }
            }

       /* }
        if(checkForDuck()) {
            path5();
        }
        else {
            move_forward(2);
            make_a_turn(rotate_dir.ROTATE_FORWARD_RIGHT, 4);
            sleep(1000);
            if (checkForDuck()) {
                path4();
            } else {
                make_a_turn(rotate_dir.ROTATE_FORWARD_LEFT, 8);
                sleep(1000);
                path6();
            }

        */
        }



        }

       // make_a_turn(rotate_dir.ROTATE_BACKWARD_LEFT, 20);
       // move_backward(43);
        //spin(30);

          /*  strafe_right(42);
            make_a_turn(rotate_dir.ROTATE_BACKWARD_LEFT, 8);
            spin(8);
            move_backward(14);
*/
    //}





       /* while (opModeIsActive()   ) {
        sleep(2000);
            if(checkForDuck()) {
                telemetry.addData("Status", "SLEPT");
                telemetry.update();
            move_forward(4);
        }
        }*/
//}



    private boolean checkForDuck() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                    telemetry.update();
                }
                for (Recognition recognition : updatedRecognitions) {
                    //recognition.getLabel());
                   if ("Duck".equals(recognition.getLabel())) {

                        telemetry.addData("DUCK", "FOUND");
                        telemetry.update();
                        return true;
                    }
                }
            }
        }

        return false;

    }


   private void moveXrail(double inches) {
        int ticks = inchesToTicksXrail(inches);
        motorXrailDrive.setDirection(DcMotor.Direction.REVERSE);
        motorXrailDrive.setTargetPosition(ticks);
        motorXrailDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorXrailDrive.setPower(0.5);
    }

    private int inchesToTicksXrail(double inches) {
       double ticks_per_rev = 383.6;
        double inches_per_rev = 4;
        return (int) (inches * ticks_per_rev / inches_per_rev);
   }

    private int getTotal_ticks(int x) {
        int total_ticks = 0;
        double motor_ticks_per_rev = 383.6;
        double wheel_gear_multiple = 2;
        double wheel_diameter_inches = 3.9370; //100mm wheel diameter
        double wheel_circumference;
        double ticks_per_wheel_inch;

        wheel_circumference = Math.PI * wheel_diameter_inches;
        ticks_per_wheel_inch = (motor_ticks_per_rev * wheel_gear_multiple) / wheel_circumference;
        //resetEncoders();
        total_ticks = (int) (x * ticks_per_wheel_inch);
        return total_ticks;
    }

    private void move_forward(int x) {
        int total_ticks = getTotal_ticks(x);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorLeftDriveDown.setDirection(DcMotor.Direction.REVERSE);
        motorRightDriveDown.setDirection(DcMotor.Direction.FORWARD);
        motorLeftDriveUp.setDirection(DcMotor.Direction.REVERSE);
        motorRightDriveUp.setDirection(DcMotor.Direction.FORWARD);

        // reset encoder counts kept by motors.
        motorLeftDriveDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightDriveUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftDriveUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightDriveUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculate the number of ticks corresponding to x inches
        // Request motor to RUN_TO_POSITION for those number of ticks

        motorLeftDriveDown.setTargetPosition(total_ticks);
        motorRightDriveDown.setTargetPosition(total_ticks);
        motorLeftDriveUp.setTargetPosition(total_ticks);
        motorRightDriveUp.setTargetPosition(total_ticks);

        motorLeftDriveDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightDriveDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftDriveUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightDriveUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftDriveDown.setPower(1);
        motorRightDriveDown.setPower(1);
        motorLeftDriveUp.setPower(1);
        motorRightDriveUp.setPower(1);

        while (opModeIsActive() && motorLeftDriveDown.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left-down", motorLeftDriveDown.getCurrentPosition() + "  busy=" + motorLeftDriveDown.isBusy());
            telemetry.addData("encoder-fwd-right-down", motorRightDriveDown.getCurrentPosition() + "  busy=" + motorRightDriveDown.isBusy());
            telemetry.addData("encoder-fwd-left-up", motorLeftDriveUp.getCurrentPosition() + "  busy=" + motorLeftDriveUp.isBusy());
            telemetry.addData("encoder-fwd-right-down", motorRightDriveUp.getCurrentPosition() + "  busy=" + motorRightDriveUp.isBusy());
            telemetry.update();
            idle();
        }

        motorLeftDriveDown.setPower(0);
        motorRightDriveDown.setPower(0);
        motorLeftDriveUp.setPower(0);
        motorRightDriveUp.setPower(0);

    }

    private void move_backward(int x) {
        int total_ticks = getTotal_ticks(x);

        // Changing the motor direction to go backward
        motorLeftDriveDown.setDirection(DcMotor.Direction.FORWARD);
        motorRightDriveDown.setDirection(DcMotor.Direction.REVERSE);
        motorLeftDriveUp.setDirection(DcMotor.Direction.FORWARD);
        motorRightDriveUp.setDirection(DcMotor.Direction.REVERSE);


        // reset encoder counts kept by motors.
        motorLeftDriveDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightDriveDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftDriveUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightDriveUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculate the number of ticks corresponding to x inches
        // Request motor to RUN_TO_POSITION for those number of ticks

        motorLeftDriveDown.setTargetPosition(total_ticks);
        motorRightDriveDown.setTargetPosition(total_ticks);
        motorLeftDriveUp.setTargetPosition(total_ticks);
        motorRightDriveUp.setTargetPosition(total_ticks);

        motorLeftDriveDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightDriveDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftDriveUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightDriveUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorLeftDriveDown.setPower(0.5);
        motorRightDriveDown.setPower(0.5);
        motorLeftDriveUp.setPower(0.5);
        motorRightDriveUp.setPower(0.5);

    }

    private void make_a_turn(rotate_dir rot_dir, int z) {
        int total_ticks = getTotal_ticks(z);

        switch (rot_dir) {
            case ROTATE_FORWARD_RIGHT:
            case ROTATE_BACKWARD_RIGHT:
                //Left front moves backward, Right front moves forward
                //Left back moves backward, Right back moves forward
                //Left front moves forward, Right front moves backward
                //Left back front moves forward, Right back moves backward
                motorLeftDriveDown.setDirection(DcMotor.Direction.REVERSE);
                motorRightDriveDown.setDirection(DcMotor.Direction.REVERSE);
                motorLeftDriveUp.setDirection(DcMotor.Direction.REVERSE);
                motorRightDriveUp.setDirection(DcMotor.Direction.REVERSE);
                break;
            case ROTATE_FORWARD_LEFT:
            case ROTATE_BACKWARD_LEFT:
                //Left front moves backward, Right front moves forward
                //Left back moves backward, Right back moves forward
                motorLeftDriveDown.setDirection(DcMotor.Direction.FORWARD);
                motorRightDriveDown.setDirection(DcMotor.Direction.FORWARD);
                motorLeftDriveUp.setDirection(DcMotor.Direction.FORWARD);
                motorRightDriveUp.setDirection(DcMotor.Direction.FORWARD);
                break;
            default:
                //Do nothing
                break;
        }

        // reset encoder counts kept by motors.
        motorLeftDriveDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightDriveDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftDriveUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightDriveUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculate the number of ticks corresponding to x inches
        // Request motor to RUN_TO_POSITION for those number of ticks

        motorLeftDriveDown.setTargetPosition(total_ticks);
        motorRightDriveDown.setTargetPosition(total_ticks);
        motorLeftDriveUp.setTargetPosition(total_ticks);
        motorRightDriveUp.setTargetPosition(total_ticks);

        motorLeftDriveDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightDriveDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftDriveUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightDriveUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftDriveDown.setPower(0.25);
        motorRightDriveDown.setPower(0.25);
        motorLeftDriveUp.setPower(0.25);
        motorRightDriveUp.setPower(0.25);

        while (opModeIsActive() && motorLeftDriveDown.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left-down", motorLeftDriveDown.getCurrentPosition() + "  busy=" + motorLeftDriveDown.isBusy());
            telemetry.addData("encoder-fwd-right-down", motorRightDriveDown.getCurrentPosition() + "  busy=" + motorRightDriveDown.isBusy());
            telemetry.addData("encoder-fwd-left-up", motorLeftDriveUp.getCurrentPosition() + "  busy=" + motorLeftDriveUp.isBusy());
            telemetry.addData("encoder-fwd-right-down", motorRightDriveUp.getCurrentPosition() + "  busy=" + motorRightDriveUp.isBusy());
            telemetry.update();
            idle();
        }

        motorLeftDriveDown.setPower(0);
        motorRightDriveDown.setPower(0);
        motorLeftDriveUp.setPower(0);
        motorRightDriveUp.setPower(0);
    }

    private void strafe_right(int x) {
        int total_ticks = getTotal_ticks(x);
        // Changing the motor direction to go backward
        motorLeftDriveDown.setDirection(DcMotor.Direction.REVERSE);
        motorRightDriveDown.setDirection(DcMotor.Direction.FORWARD);
        motorLeftDriveUp.setDirection(DcMotor.Direction.REVERSE);
        motorRightDriveUp.setDirection(DcMotor.Direction.FORWARD);


        // reset encoder counts kept by motors.
        motorLeftDriveDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightDriveDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftDriveUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightDriveUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculate the number of ticks corresponding to x inches
        // Request motor to RUN_TO_POSITION for those number of ticks

        motorLeftDriveDown.setTargetPosition(total_ticks * -1);
        motorRightDriveDown.setTargetPosition(total_ticks);
        motorLeftDriveUp.setTargetPosition(total_ticks);
        motorRightDriveUp.setTargetPosition(total_ticks * -1);

        motorLeftDriveDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightDriveDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftDriveUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightDriveUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftDriveDown.setPower(0.25);
        motorRightDriveDown.setPower(0.25);
        motorLeftDriveUp.setPower(0.25);
        motorRightDriveUp.setPower(0.25);

        //opModeIsActive()
        while (motorLeftDriveDown.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left-down", motorLeftDriveDown.getCurrentPosition() + "  busy=" + motorLeftDriveDown.isBusy());
            telemetry.addData("encoder-fwd-right-down", motorRightDriveDown.getCurrentPosition() + "  busy=" + motorRightDriveDown.isBusy());
            telemetry.addData("encoder-fwd-left-up", motorLeftDriveUp.getCurrentPosition() + "  busy=" + motorLeftDriveUp.isBusy());
            telemetry.addData("encoder-fwd-right-down", motorRightDriveUp.getCurrentPosition() + "  busy=" + motorRightDriveUp.isBusy());
            telemetry.update();
            idle();
        }

        motorLeftDriveDown.setPower(0);
        motorRightDriveDown.setPower(0);
        motorLeftDriveUp.setPower(0);
        motorRightDriveUp.setPower(0);
    }

    private void strafe_left(int x) {
        int total_ticks = getTotal_ticks(x);
        // Changing the motor direction to go backward
        motorLeftDriveDown.setDirection(DcMotor.Direction.FORWARD);
        motorRightDriveDown.setDirection(DcMotor.Direction.REVERSE);
        motorLeftDriveUp.setDirection(DcMotor.Direction.FORWARD);
        motorRightDriveUp.setDirection(DcMotor.Direction.REVERSE);


        // reset encoder counts kept by motors.
        motorLeftDriveDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightDriveDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftDriveUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightDriveUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Calculate the number of ticks corresponding to x inches
        // Request motor to RUN_TO_POSITION for those number of ticks

        motorLeftDriveDown.setTargetPosition(total_ticks*-1 );
        motorRightDriveDown.setTargetPosition(total_ticks) ;
        motorLeftDriveUp.setTargetPosition(total_ticks);
        motorRightDriveUp.setTargetPosition(total_ticks*-1);

        motorLeftDriveDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightDriveDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftDriveUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightDriveUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeftDriveDown.setPower(0.25);
        motorRightDriveDown.setPower(0.25);
        motorLeftDriveUp.setPower(0.25);
        motorRightDriveUp.setPower(0.25);

        //opModeIsActive()
        while (motorLeftDriveDown.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left-down", motorLeftDriveDown.getCurrentPosition() + "  busy=" + motorLeftDriveDown.isBusy());
            telemetry.addData("encoder-fwd-right-down", motorRightDriveDown.getCurrentPosition() + "  busy=" + motorRightDriveDown.isBusy());
            telemetry.addData("encoder-fwd-left-up", motorLeftDriveUp.getCurrentPosition() + "  busy=" + motorLeftDriveUp.isBusy());
            telemetry.addData("encoder-fwd-right-down", motorRightDriveUp.getCurrentPosition() + "  busy=" + motorRightDriveUp.isBusy());
            telemetry.update();
            idle();
        }

        motorLeftDriveDown.setPower(0);
        motorRightDriveDown.setPower(0);
        motorLeftDriveUp.setPower(0);
        motorRightDriveUp.setPower(0);
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
      //  checkForDuck();

    }
    private void tilt(){
        servoTilterDrive.setDirection(Servo.Direction.FORWARD);
        servoTilterDrive.setPosition(1);
         }

         private void spin(int x){
             int total_ticks = getTotal_ticks(x);
        motorSpinnerDrive.setDirection(DcMotor.Direction.FORWARD);
        motorSpinnerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSpinnerDrive.setDirection(DcMotor.Direction.FORWARD);
        motorSpinnerDrive.setTargetPosition(total_ticks);
        motorSpinnerDrive.setMode(DcMotor.RunMode. RUN_TO_POSITION);
        motorSpinnerDrive.setPower(.5);


         }

public void path1() {
    moveXrail(8);
        make_a_turn(rotate_dir.ROTATE_FORWARD_RIGHT,12);
        move_forward(20);
        tilt();
        //method for tilting servo to drop
}
public void path2(){
    moveXrail(18);
  make_a_turn(rotate_dir.ROTATE_FORWARD_RIGHT, 9);
  move_forward(20);
    tilt();
    //method for servo to drop
}
    public void path3(){
        //method for picking up blocks
        moveXrail(28);
        move_forward(20);
        tilt();
        //method for servo to drop
    }
    public void path4(){
        moveXrail(8);
       make_a_turn(rotate_dir.ROTATE_FORWARD_LEFT,12);
       move_forward(20);
        tilt();
    }
    private void path5(){
        moveXrail(18);
       make_a_turn(rotate_dir.ROTATE_FORWARD_LEFT, 9);
       move_forward(16);
        tilt();
    }
    private void path6(){
        moveXrail(4);
        make_a_turn(rotate_dir.ROTATE_FORWARD_LEFT, 2);
        move_forward(17);
        tilt();
    }
    }
