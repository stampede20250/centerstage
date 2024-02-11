package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TF_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration in correct hub
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FLM");    //312 motor
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BLM");     //312 motor
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FRM");   //312 motor
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BRM");    //312 motor
        DcMotor lift = hardwareMap.dcMotor.get("lift");             //linear lift Motor
        DcMotor arm = hardwareMap.dcMotor.get("arm");               //Arm Motor
        //servos
        Servo plane = hardwareMap.servo.get("plane"); // servo for plane release
        Servo hook = hardwareMap.servo.get("hook"); //   servo to rotate to hang at end game

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData(">", "Good Luck from Team Stampede 20250");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

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

            // buttons are the following:
            // circle/B
            // cross/A √
            // triangle/Y √
            // square/X √
            // left_bumper
            // right_bumper
            // dpad
            // touchpad √
            // RightStickButton
            // LeftStickButton

            // below are gamepad 1 controls
            // you need to find what setPositions are needed - use the Servo Test teleop code to find positions and enter in the (.50)

            if (gamepad1.touchpad) // touchpad, (less chance of accidently clicking)  if your controller does not have touchpad (ps4 compatible, use another button from above) - there are √ to already used buttons, choose an available one.
                plane.setPosition(.50);
            //
            if (gamepad1.b) //Button A
                hook.setPosition(.50);
            // if you want to bring hook back, need another button, or accept as it is if you make a mistake.

            //this code is to raise and lower lift
            if (gamepad2.dpad_up) {
                lift.setPower(-1);
            } else if (gamepad2.dpad_down) {
                lift.setPower(1);
            } else {             // Motors stops when let go of the controls
                lift.setPower(0); //viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }


            // below are gamepad 2 controls

            if (gamepad2.dpad_up) {
                arm.setPower(-1);
            } else if (gamepad2.dpad_down) {
                arm.setPower(1);
            } else {             // Motors stops when let go of the controls
                arm.setPower(0); //viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


        }
    }
}







