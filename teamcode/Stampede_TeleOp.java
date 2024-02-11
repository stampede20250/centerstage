package org.firstinspires.ftc.teamcode;

import android.graphics.BlendMode;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Stampede_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration in correct hub
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FLM");  //312 motor
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BLM");  //312 motor
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FRM"); //312 motor
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BRM"); //312 motor
        DcMotor viper = hardwareMap.dcMotor.get("viper"); //viper slides
        DcMotor lift = hardwareMap.dcMotor.get("lift"); //worm drive lift
        Servo plane = hardwareMap.servo.get("plane"); // servo for plane release      EH-0
        Servo Lflip = hardwareMap.servo.get("Lflip"); // flippers                        controlhub 0
        Servo Rflip = hardwareMap.servo.get("Rflip"); //flippers                         controlhub 1
        Servo rotate = hardwareMap.servo.get("rotate"); //roatate the cage            EH-1
        DigitalChannel touchT = hardwareMap.digitalChannel.get("touchT");
        DigitalChannel touchB = hardwareMap.digitalChannel.get("touchB");
        //DistanceSensor see = hardwareMap.get(DistanceSensor.class, "see");
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) see;


        //insert distance sensor here

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Lflip.setPosition(.45);
        Rflip.setPosition(.75);
        rotate.setPosition(.79);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            // If the distance in centimeters is less than 10, set the power to 0.3
            //if (see.getDistance(DistanceUnit.INCH) < 10) {
            //    frontLeftMotor.setPower(.3);
            //    backLeftMotor.setPower(.3);
            //    frontRightMotor.setPower(.3);
            //    backRightMotor.setPower(.3);
            //} else {  // Otherwise, stop the motor
            //    frontLeftMotor.setPower(0);
            //    backLeftMotor.setPower(.0);
            //    frontRightMotor.setPower(0);
            //    backRightMotor.setPower(0);

            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed - used to be -gamepad1.left_stick_y; revert back if not expected functions.
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                // below are gamepad 1 controls
                // buttons are the following:
                // circle/B √
                // cross/A √
                // triangle/Y √
                // square/X √
                // left_bumper √
                // right_bumper √
                // dpad
                // touchpad √
                // RightStickButton
                // LeftStickButton



            if (gamepad1.touchpad) //button
                plane.setPosition(1.00);
            //these open flippers one by one
            if (gamepad1.cross) //Button A
                Lflip.setPosition(.50);
            if (gamepad1.circle) //Button B
                Rflip.setPosition(.50);
            //these close flippers one by one
            if (gamepad1.square) //Button X
                Lflip.setPosition(.45);
            if (gamepad1.triangle) //Button Y
                Rflip.setPosition(.75);

            // below are gamepad 2 controls
            // If the either touch sensor is pressed, stop the motor

            if (gamepad2.right_bumper && touchT.getState()) {    // If the touch sensor is pressed, stop the motor
                    lift.setPower(-1.0);
                } else if (gamepad2.left_bumper && touchB.getState()) {   // If the touch sensor is pressed, stop the motor
                    lift.setPower(1.0);
                } else {
                    lift.setPower(0);
                }
                if (gamepad2.dpad_up) {
                    viper.setPower(-1);
                } else if (gamepad2.dpad_down) {
                    viper.setPower(1);
                } else {             // Motors stops when let go of the controls
                    viper.setPower(0); //viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }

                if (gamepad2.cross) //button Y
                    rotate.setPosition(0.79); //this is the gripper to land at bottom ready to go.
                if (gamepad2.triangle) //Button B
                    rotate.setPosition(.19); //this is angle set for the board

                else {

                }

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

                //telemetry.addData("deviceName", see.getDeviceName() );
                //telemetry.addData("range", String.format("%.01f in", see.getDistance(DistanceUnit.INCH)));

                // Rev2mDistanceSensor specific methods.
                //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
                //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

                telemetry.update();

            }
        }
    }







