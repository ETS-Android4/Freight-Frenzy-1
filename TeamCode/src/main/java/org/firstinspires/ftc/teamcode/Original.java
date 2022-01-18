package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Original", group = "Linear Opmode")
//@Disabled
public class Original extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorLeftUp = null;
    private DcMotor motorLeftDown = null;
    private DcMotor motorRightUp = null;
    private DcMotor motorRightDown = null;
    private DcMotor motorXrail = null;
    private DcMotor motorSpinner = null;
    private DcMotor leftSpinner = null;
    private CRServo servoTilter = null;
    //private Servo servoTilter= null;
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
        servoTilter = hardwareMap.get(CRServo.class, "tilter_servo");
        leftSpinner =hardwareMap.get(DcMotor.class, "spinner_left");
       // servoTilter = hardwareMap.get(Servo.class, "tilter_servo");

        double motorSpeed = 1;
        double frontspeed= .5;
        double backspeed = .44;
        double xrailSpeed = .5;
        //double position = servoTilter.getPosition();
        //double increment = 0.01;
        //double Max_pos = 0.3;
        //double Min_pos = 0.0;
        //double pos = 0.5;

        motorXrail.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLeftUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorLeftUp.setDirection(DcMotor.Direction.FORWARD);
        motorRightUp.setDirection(DcMotor.Direction.REVERSE);
        motorLeftDown.setDirection(DcMotor.Direction.FORWARD);
        motorRightDown.setDirection(DcMotor.Direction.REVERSE);
        motorXrail.setDirection(DcMotor.Direction.REVERSE);
        motorSpinner.setDirection(DcMotor.Direction.FORWARD);
        leftSpinner.setDirection(DcMotor.Direction.REVERSE);
        servoTilter.setDirection(CRServo.Direction.REVERSE);
        //servoTilter.setDirection(Servo.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                motorSpeed = 1;
            }
            if (gamepad1.left_bumper) {
                motorSpeed = 0.5;
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
           ///*
            if (gamepad1.left_stick_y > 0.05 || gamepad1.left_stick_y < -0.05 ) {
                motorLeftUp.setPower(motorSpeed * gamepad1.left_stick_y);
                motorRightUp.setPower(motorSpeed * gamepad1.left_stick_y);
                motorLeftDown.setPower(motorSpeed * gamepad1.left_stick_y);
                motorRightDown.setPower(motorSpeed * gamepad1.left_stick_y);
            }

            motorXrail.setPower(xrailSpeed * gamepad2.right_stick_y);
            motorSpinner.setPower(motorSpeed * Math.abs(gamepad2.left_stick_x));
            leftSpinner.setPower(motorSpeed * Math.abs(gamepad2.left_stick_x));
           // */
            double strafeSpeed = .8;
            double strafeBad = 1;
            if(!(gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.left_stick_y > 0.05 || gamepad1.left_stick_y < -0.05  )) {
                motorLeftDown.setPower(0);
                motorRightDown.setPower(0);
                motorLeftUp.setPower(0);
                motorRightUp.setPower(0);
            }
            if (gamepad1.dpad_left) {
                //moves mecanum wheels left and right
                motorLeftDown.setPower(backspeed * -1);
                motorRightDown.setPower(frontspeed);
                motorLeftUp.setPower(frontspeed);
                motorRightUp.setPower(frontspeed * -1);
            }

            if (gamepad1.dpad_right) {
                //moves mecanum wheels left and right
                motorLeftDown.setPower(.47);
                motorRightDown.setPower(frontspeed * -1);
                motorLeftUp.setPower(frontspeed * -1);
                motorRightUp.setPower(frontspeed);
            }

            //turn robot clockwise
            if (gamepad1.right_stick_x != 0) {
                motorLeftDown.setPower(motorSpeed * gamepad1.right_stick_x * -1);
                motorRightDown.setPower(motorSpeed * gamepad1.right_stick_x);
                motorLeftUp.setPower(motorSpeed * gamepad1.right_stick_x * -1);
                motorRightUp.setPower(motorSpeed * gamepad1.right_stick_x);

            }

            //servoTilter.getPosition()


            if (gamepad2.dpad_down) {

                //servoTilter.setPosition(position);
               servoTilter.setPower(-1);
                //pos = pos - 0.05;
               // servoTilter.setPosition(0); // 0 points to ground
            }
            else if (gamepad2.dpad_up) {
                servoTilter.setPower(1);
                //pos = pos + 0.05;
              //  servoTilter.setPosition(1); // 1 points to wall
            }
            else {
                servoTilter.setPower(0);
            }
            /*if (gamepad2.dpad_left) {
                servoTilter.setPower(1);
              // servoTilter.setPosition(.5);
            }
            if (gamepad2.dpad_right) {
                servoTilter.setPower(0);
            }*/
           // servoTilter.setPower(0);

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
          // telemetry.addData("tilter_servo", servoTilter.getPosition());
            telemetry.addData("motorSpeedback" ,  motorLeftDown.getPower());
            telemetry.addData("motorspeedfront" , motorLeftUp.getPower());
            telemetry.addData("servopower" , servoTilter.getPower());
            telemetry.update();
            idle();
        }
    }
}
