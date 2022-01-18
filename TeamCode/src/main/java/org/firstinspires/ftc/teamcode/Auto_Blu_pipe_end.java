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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Auto_Blu_pipe_end", group = "Original")
public class Auto_Blu_pipe_end extends LinearOpMode {

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
    private DcMotor motorSpinnerDrive_left = null;
    private CRServo servoTilterDrive = null;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

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
            tfod.setZoom(2.8, 16.0 / 9.0);
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
        motorSpinnerDrive_left = hardwareMap.get(DcMotor.class, "spinner_left");
        servoTilterDrive = hardwareMap.get(CRServo.class, "tilter_servo");

        double motorSpeed = 1;
        double xrailSpeed = .5;
        //double position = servoTilterDrive.getPosition();
        double increment = 0.01;
        double Max_pos = .3;
        double Min_pos = 0.0;
        int count = 0;
        long starttimer = System.currentTimeMillis();
        long timepassed = System.currentTimeMillis() - starttimer;


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
        motorSpinnerDrive.setDirection(DcMotor.Direction.FORWARD);
        servoTilterDrive.setDirection(CRServo.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        tilt(700, .6);
        move_forward(6);
        sleep(1600);





// Blue side near pipe end

        if (checkForDuck()) {
            path2();
        } else {
            strafe_right(12);
            sleep(1000);
            if (checkForDuck()) {
                path3();
            } else {
                sleep(1000);
                path1();
            }
        }



// Blue side near Spinner end
        /*

            if (checkForDuck()) {
                path5();
            } else {
                strafe_left(11);
                sleep(1000);
                if (checkForDuck()) {
                    path4();
                } else {

                    sleep(1000);
                    path6();
                }
            }
*/
// Red side near Spinner end
/*
        if (checkForDuck()) {
            path8();
        } else {
            strafe_right(12);
            sleep(1200);
            if (checkForDuck()) {
                path9();
            } else {
                sleep(1000);
                path7();
            }
        }

*/
// Red side near pipe end
/*
        if (checkForDuck()) {
            path11();
        } else {
            strafe_left(10);
            sleep(1000);
            if (checkForDuck()) {
                path12();
            } else {

                sleep(1000);
                path10();
            }
            */

        }





//}






    //}





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
        motorXrailDrive.setDirection(DcMotor.Direction.FORWARD);
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



    private void tilt(long milliseconds, double power){
        servoTilterDrive.setDirection(CRServo.Direction.FORWARD);
        long starttime = System.currentTimeMillis();
        servoTilterDrive.setPower(power);
        while((System.currentTimeMillis() - starttime) < milliseconds) {

        }
        servoTilterDrive.setPower(0);

    }



         private void spin(int x){
            // int total_ticks = getTotal_ticks(x);
        motorSpinnerDrive.setDirection(DcMotor.Direction.FORWARD);
        motorSpinnerDrive_left.setDirection(DcMotorSimple.Direction.FORWARD);
       // motorSpinnerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //   motorSpinnerDrive.setTargetPosition(total_ticks);
       // motorSpinnerDrive.setMode(DcMotor.RunMode. RUN_TO_POSITION);
             motorSpinnerDrive_left.setPower(.7);
             motorSpinnerDrive.setPower(.7);
             while (opModeIsActive() && motorSpinnerDrive.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
             {
                 telemetry.addData("encoder-fwd-left-down", motorSpinnerDrive.getCurrentPosition() + "  busy=" + motorSpinnerDrive.isBusy());
                 telemetry.update();
                 idle();
             }
             sleep(4500);
        motorSpinnerDrive.setPower(0);
        motorSpinnerDrive_left.setPower(0);
         }
    private void spinop(int x){
        // int total_ticks = getTotal_ticks(x);
        motorSpinnerDrive.setDirection(DcMotor.Direction.REVERSE);
        motorSpinnerDrive_left.setDirection(DcMotorSimple.Direction.REVERSE);
        // motorSpinnerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //   motorSpinnerDrive.setTargetPosition(total_ticks);
        // motorSpinnerDrive.setMode(DcMotor.RunMode. RUN_TO_POSITION);
        motorSpinnerDrive_left.setPower(.7);
        motorSpinnerDrive.setPower(.7);
        while (opModeIsActive() && motorSpinnerDrive.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left-down", motorSpinnerDrive.getCurrentPosition() + "  busy=" + motorSpinnerDrive.isBusy());
            telemetry.update();
            idle();
        }
        sleep(4500);
        motorSpinnerDrive.setPower(0);
        motorSpinnerDrive_left.setPower(0);
    }


public void path1() {
    moveXrail(11);
       // make_a_turn(rotate_dir.ROTATE_FORWARD_RIGHT,12);
    strafe_right(15);
        move_forward(13);
        tilt(50,.2);
    sleep(400);
    tilt(150,.2);
    move_backward(15);
    strafe_right(51);
    spin(200);
    move_forward(20);
        //method for tilting servo to drop
}
public void path2(){
        sleep(100);
    moveXrail(19);
 // make_a_turn(rotate_dir.ROTATE_FORWARD_RIGHT, 9);
    strafe_right(25);
    sleep(600);
    move_forward(14);
    tilt(150,.2);
    move_backward(15);
    strafe_right(52);
    spin(200);
    move_forward(20);

    //method for servo to drop
}
    public void path3(){
        //method for picking up blocks
        moveXrail(33);
        strafe_right(15);
        move_forward(15);
        tilt(250,.2);
        sleep(400);
        move_backward(15);
        strafe_right(52);
        spin(10000);
        move_forward(20);
        //method for servo to drop
    }
    public void path4(){
        tilt(200, .1);
        moveXrail(11);
        tilt(200, .1);
        // make_a_turn(rotate_dir.ROTATE_FORWARD_RIGHT,12);
        strafe_left(15);
        move_forward(13);
        tilt(50,.2);
        sleep(400);
        tilt(250,.2);
        move_backward(15);
        strafe_right(56);
        spin(200);
        move_forward(19);
    }
    private void path5(){
        sleep(100);
        tilt(200, .1);
        moveXrail(19);
        tilt(200, .1);
        // make_a_turn(rotate_dir.ROTATE_FORWARD_RIGHT, 9);
        strafe_left(25);
        sleep(600);
        move_forward(14);
        tilt(250,.2);
        move_backward(15);
        strafe_right(52);
        spin(200);
        move_forward(20);

    }
    private void path6(){
        tilt(200, .1);
        sleep(300);
        moveXrail(33);
        tilt(200, .1);
        strafe_left(13);
        move_forward(15);
        tilt(250,.2);
        sleep(400);
        move_backward(15);
        strafe_right(52);
        spin(10000);
        move_forward(18);
    }
    private void path7(){
        sleep(200);
        moveXrail(11);
        strafe_right(15);
        move_forward(13);
        tilt(50,.2);
        sleep(800);
        tilt(150,.2);
        move_backward(14);
        strafe_left(50);
        spinop(200);
        move_forward(18);
    }
    private void path8(){
        sleep(200);
        moveXrail(19);
        // make_a_turn(rotate_dir.ROTATE_FORWARD_RIGHT, 9);
        strafe_right(24);
        sleep(800);
        move_forward(15);
        tilt(150,.2);
        move_backward(14);
        strafe_left(50);
        spinop(200);
        move_forward(18);
    }
    private void path9(){
        sleep(300);
        moveXrail(33);
        strafe_right(15);
        move_forward(13);
        sleep(800);
        tilt(250,.2);
        sleep(500);
        move_backward(14);
        strafe_left(50);
        spinop(10000);
        move_forward(20);
    }
    private void path10(){
        moveXrail(10);
        tilt(100,.1);
        // make_a_turn(rotate_dir.ROTATE_FORWARD_RIGHT,12);
        strafe_left(18);
        sleep(200);
        move_forward(12);
        tilt(50,.2);
        sleep(400);
        tilt(150,.2);
        move_backward(19);
        strafe_left(50);
        move_backward(4);
        spinop(200);
        move_forward(20);

    }
    private void path11(){
        sleep(100);
        moveXrail(19);
        strafe_left(30);
        sleep(600);
        move_forward(12);
        tilt(200,.2);
        move_backward(17);
        strafe_left(50);
        move_backward(4);
        spinop(200);
        move_forward(20);

    }
    private void path12(){
        moveXrail(33);
        strafe_left(17);
        sleep(200);
        move_forward(12);
        sleep(200);
        tilt(250,.2);
        sleep(400);
        move_backward(19);
        strafe_left(50);
        move_backward(4);
        spinop(10000);
        move_forward(20);

    }
    }
