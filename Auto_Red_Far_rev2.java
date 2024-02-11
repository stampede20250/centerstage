package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

//Wish List to finish
// Camera to see spike
// drop pixel on spike (33 percent to succeed if choose only middle if cannot get camera going)
// rotate and move foward to back panel and hopefully sensor works if needed
// back to back panel
// rotate pixel holder
// lift arm
// drop pixel on board

// 1/8/24 fixed case 3, 4 and 5 toward the backdrop on the red far red side

@Autonomous(name="Auto_Red_Far_rev2", group="Phase 1")
//@Disabled
public class Auto_Red_Far_rev2 extends LinearOpMode {

// Camera section for variable initializations
    private OpenCvCamera webcam;
    private String position;

    String TPROP_Detected = "";

// End Camera section for variable initializations
    private final ElapsedTime runtime = new ElapsedTime();

// Start Apriltag section for variable initializations
    final double DESIRED_DISTANCE = 0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.02 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = 5;     // Choose the tag you want to approach or set to -1 for ANY tag.
    // removed "final" in this line above to allow flex the numbers
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
//  End AprilTag section for variable initializations

// Motor section for variable initializations
    DcMotor frontLeftMotor; //FLM
    DcMotor backLeftMotor;  //BLM
    DcMotor frontRightMotor; //FRM
    DcMotor backRightMotor;  //BRM
    DcMotor lift;

// End Motor section for variable initializations

// Touch Sensors section for variable initializations
    DigitalChannel touchB, touchT;

// End Touch Sensors section for variable initializations

// Initialize State for all switch:case section for variable initializations
    int state = 0;

    boolean next_step = false;

// End State for all switch:case for variable initializations

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//************************************ Camera Detection of Apriltag *********************
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        //initAprilTag();


//************************************ Camera Detection of Team Prop Red *********************

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        // OpenCV pipeline
        ContourPipeline2_6_Yellow_4Red pipeline = new ContourPipeline2_6_Yellow_4Red(telemetry);
        webcam.setPipeline(pipeline);
        //GPU Acclerating to get better render. Reduce CPU load and battery loading
        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

        // Webcam streaming
        webcam.openCameraDeviceAsync(
                new AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                    }
                    @Override
                    public void onError(int errorCode) {
                    }
                });

        sleep(3000);

        TPROP_Detected = pipeline.getLocation();

//********************************************************************************************

        frontLeftMotor = hardwareMap.dcMotor.get("FLM");  //312 motor
        backLeftMotor = hardwareMap.dcMotor.get("BLM");  //312 motor
        frontRightMotor = hardwareMap.dcMotor.get("FRM"); //312 motor
        backRightMotor = hardwareMap.dcMotor.get("BRM"); //312 motor

        lift = hardwareMap.dcMotor.get("lift"); //worm gear

        touchB = hardwareMap.digitalChannel.get("touchB");
        touchT = hardwareMap.digitalChannel.get("touchT");
        Servo rotate = hardwareMap.servo.get("rotate");
        Servo Lflip = hardwareMap.servo.get("Lflip"); // flippers                        controlhub 0
        Servo Rflip = hardwareMap.servo.get("Rflip"); //flippers

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//***************************** End to initialize to configuration on the RC *******************

//***************************** To press "init" to retain the pixels yellow and purple on the arm ****
        Lflip.setPosition(.52);  //to close the pixels
        Rflip.setPosition(.78);  //to close the pixels prior to auto

//***************************** Telemetry to indicate that the code is start ************
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("position", pipeline.getLocation());
        //telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("TSE_Detector --->", TPROP_Detected);
        telemetry.update();

        waitForStart();
        //runtime.reset();

        //telemetry.addData("TSE_Detector --->", TPROP_Detected);
        //telemetry.update();

// When pressed start in Driver's hub this code starts
// run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {
            telemetry.addData("position", pipeline.getLocation());
            telemetry.addData("TSE_Detector --->", TPROP_Detected);
            telemetry.addData("DESIRED_TAG_ID --->", DESIRED_TAG_ID);
            telemetry.update();

            switch (state) {
                case 0:
                    rotate.setPosition(0.0);
                    // to turn off the camera after detecting the team prop
                    webcam.stopStreaming();
                    webcam.stopRecordingPipeline();

                    if (TPROP_Detected == "CENTER"){
                        state = 1;
                        DESIRED_TAG_ID = 5;
                    } else if (TPROP_Detected == "LEFT"){
                        state = 14;
                        DESIRED_TAG_ID = 4;
                    }
                    else {
                        state = 21;  // for RIGHT
                        DESIRED_TAG_ID = 6;
                    }

                    break;

/* CENTER */    case 1:  //the center detected and drop off purple pixel at this spike line
                    lift_control_down(3500);

                    rotate.setPosition(0.0); //down ready to drop off the purple pixel

                    drive_motor_move(2100, 2100, 1.0);

                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 2:// drop off the purple pixel
                    sleep(200);
                    Rflip.setPosition(.50);
                    //sleep(200);
                    state++;
                    break;
                case 3: //after dropping off the pixel and drive a few clicks
                    //rotate.setPosition(0.94); //Rotation up to hold a yellow pixel

                    drive_motor_move(200, 200, 1.0);

                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
/* two cycles */    case 4: //turning to go under the stage door
                    drive_motor_move(1045, -1045, 1.0); //rotate to the right
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 5: //Go under the stage door
                    drive_motor_move(2900, 2900, 1.0);

                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 6: //move blindly toward the center line to the backdrop

                    drive_motor_sideways(-1800, 1.0); //positive value to the left, negative to right

                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 7:
                    //initAprilTag();
                    //drivetoApriltag(); // added to see if this works
                    //sleep(1000);
                    rotate.setPosition(0.94);
                    lift_control_up(-7000); //including the state++ due to touch sensor if turned on

                    break;
                case 8:
                    drive_motor_move(1400, 1400, 0.7);
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 9:
                    Lflip.setPosition(.57);  //to open as to drop off at the backdrop
                    sleep(300);
                    state ++;
                    break;
                case 10:  // lift move opposite to touch sensor for the backstage
                    drive_motor_move(-200,-200,1.0);
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 11:  //move sideways to the left away from the backdrop
                    drive_motor_sideways(1500, 1.0); //postive to left
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 12:
                    lift_control_down(7000);
                    rotate.setPosition(0.0);
                    if (!touchB.getState()){
                        state++;
                    }
                    break;
                case 13:  //turn away from the backdrop
                    drive_motor_move(300, -300, 1.0);
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state = 30;
                    }
                    break;
    /* LEFT */  case 14: // LEFT spike line only - start of the movement for the left side spike mark (First CENTER then this LEFT)
                    //drive_motor_sideways(800, 1.0);
                    drive_motor_move(300, 300, 1.0);

                    lift_control_down(3500);

                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 15: // LEFT section only - start of the movement for the left side spike mark (First CENTER then this LEFT)
                    drive_motor_sideways(700, 1.0);
                    //drive_motor_move(300, 300, 1.0);
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 16:  //the center detected and drop off purple pixel at this area
                    //rotate.setPosition(0.0); //down ready to drop off the purple pixel

                    drive_motor_move(1200, 1200, 1.0);

                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 17:// drop off the purple pixel
                    sleep(200);
                    Rflip.setPosition(.50);
                    //sleep(200);
                    state++;
                    break;
                case 18: //after dropping off the pixel and drive a few clicks
                    //rotate.setPosition(0.94); //Rotation up to hold a yellow pixel

                    drive_motor_move(600, 600, 1.0);

                    if (drive_motor_position_done()) {
                       drive_motor_stop_reset();
                       state++;  //same routine as in CENTER
                    }
                    break;
                case 19: //turning to go under the stage door
                    drive_motor_move(1045, -1045, 1.0); //rotate to the right
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 20: //Go under the stage door
                    drive_motor_move(3200, 3200, 1.0);

                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
/* go to CASE 6 */      state = 6; //go back the same routine after passing the stage door
                    }
                    break;
/* RIGHT*/      case 21: //this is for RIGHT at spike line
                    lift_control_down(3500);

                    rotate.setPosition(0.0);

                    drive_motor_move(1200, 1200, 1.0);
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 22: //this is for RIGHT at spike line
                    drive_motor_move(-1040, 1040, 1.0);  //move the bot to the left
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 23: //this is for RIGHT at spike line
                    drive_motor_move(-80, -80, 1.0);
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 24:// drop off the purple pixel
                    sleep(200);
                    Rflip.setPosition(.50);
                    //sleep(200);
                    state++;
                    break;
                case 25: //
                    drive_motor_move(200, 200, 1.0);
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 26: //
                    drive_motor_move(1040, -1040, 1.0);  //turn the bot to right
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
                        state++;
                    }
                    break;
                case 27: //this is for RIGHT at spike line
                    drive_motor_move(1200, 1200, 1.0);
                    if (drive_motor_position_done()) {
                        drive_motor_stop_reset();
/* Go to CASE 4*/       state = 4;
                    }
                    break;
            default:
            }
            telemetry.addData("State: --> ", state);
            telemetry.update();
        }
    }
    private void drive_motor_move(int leftPos, int rightPos, double power){
        backLeftMotor.setTargetPosition(leftPos);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setPower(power);
        frontLeftMotor.setTargetPosition(leftPos);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower(power);
        frontRightMotor.setTargetPosition(rightPos);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setPower(power);
        backRightMotor.setTargetPosition(rightPos);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setPower(power);
    }
    private void drive_motor_sideways(int position, double power){
        backLeftMotor.setTargetPosition (position);
        backLeftMotor.setMode (DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setPower (power);
        frontLeftMotor.setTargetPosition (-position);
        frontLeftMotor.setMode (DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setPower (power);
        frontRightMotor.setTargetPosition (position);
        frontRightMotor.setMode (DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setPower (power);
        backRightMotor.setTargetPosition (-position);
        backRightMotor.setMode (DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setPower (power);
    }
    private void drive_motor_stop_reset() {
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public boolean drive_motor_position_done() {
        return !backRightMotor.isBusy() && !frontRightMotor.isBusy() && !frontLeftMotor.isBusy() && !backLeftMotor.isBusy();
    }
    private void lift_control_down(int lower){
        if (!touchB.getState()){
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //sleep(200);
            //state++;
        }
        else {
            lift.setTargetPosition(lower);  //move forward to the target zone
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(0.9);
        }
    }
    private void lift_control_up(int upper){
        if (!touchT.getState()){
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(200);
            state++;
        }
        else {
            lift.setTargetPosition(upper);  //move forward to the target zone
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(0.9);
        }
    }

//*************** Private Void for Apriltag ******************************

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        backLeftMotor.setPower(leftBackPower);
        frontRightMotor.setPower(rightFrontPower);
        frontLeftMotor.setPower(leftFrontPower);
        backRightMotor.setPower(rightBackPower);

    }
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    private void drivetoApriltag() {
        boolean targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further (select first/leftmost found tag)
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        //if (gamepad1.left_bumper && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            double drive;
            double turn;
            double strafe;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        //} else {

            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
        //    drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
        //    strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
        //    turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
        //    telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        //}
        telemetry.update();

        // Apply desired axes motions to the drivetrain.
        moveRobot(drive, strafe, turn);
        sleep(10);
    }
//*************** END Private Void for Apriltag ******************************
}
