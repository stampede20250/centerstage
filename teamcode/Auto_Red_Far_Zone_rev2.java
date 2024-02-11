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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//Wish List to finish
// Camera to see spike
// drop pixel on spike (33 percent to succeed if choose only middle if cannot get camera going)
// rotate and move foward to back panel and hopefully sensor works if needed
// back to back panel
// rotate pixel holder
// lift arm
// drop pixel on board

// 1/8/24 fixed case 3, 4 and 5 toward the backdrop on the red far red side

@Autonomous(name="Auto_Red_Far_Zone_rev2", group="Phase 1")
//@Disabled
public class Auto_Red_Far_Zone_rev2 extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    DcMotor frontLeftMotor; //FLM
    DcMotor backLeftMotor;  //BLM
    DcMotor frontRightMotor; //FRM
    DcMotor backRightMotor;  //BRM

    DcMotor lift;

    DigitalChannel touchB, touchT;

    Servo Lflip;
    Servo Rflip;

    int state = 0;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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


        Lflip.setPosition(.45);
        Rflip.setPosition(.75);

        /** Wait for the game to begin */
        telemetry.addData(">", "Move Forth and get points!");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)



        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            switch (state) {
                case 0:
                    int moveForward = 2400;  //
                    backLeftMotor.setTargetPosition(moveForward);  //move forward to the target zone
                    backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backLeftMotor.setPower(0.7);

                    frontLeftMotor.setTargetPosition(moveForward);
                    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontLeftMotor.setPower(0.7);

                    frontRightMotor.setTargetPosition(moveForward);
                    frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontRightMotor.setPower(0.7);

                    backRightMotor.setTargetPosition(moveForward);
                    backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backRightMotor.setPower(0.7);

                    sleep(2000);

                    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    sleep(500);

                    state++;
                    break;

                case 1:
                    int turn = 1040;  // 1100 turn too much 1/9/24
                    int turn2 = -1040;
                    backLeftMotor.setTargetPosition(turn);  //turn right
                    backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backLeftMotor.setPower(.5);

                    frontLeftMotor.setTargetPosition(turn);
                    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontLeftMotor.setPower(.5);

                    frontRightMotor.setTargetPosition(turn2);
                    frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontRightMotor.setPower(.5);

                    backRightMotor.setTargetPosition(turn2);
                    backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backRightMotor.setPower(.5);

                    sleep(1500);

                    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    sleep(500);

                    state++;
                    break;

                case 2:  //move the lift to lower level to get under the stage door
                    int lower = 500;

                    if (!touchB.getState()){
                        lift.setPower(0);
                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        sleep(1000);
                        state++;
                    }
                    else {
                        lift.setTargetPosition(lower);  //move forward to the target zone
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(0.7);
                        rotate.setPosition(0.21);
                        //sleep(1000);
                    }
                    break;

                case 3:  //pass under the stage door
                int moveForward2 = 3600;

                backLeftMotor.setTargetPosition(moveForward2);  //move forward to the target zone
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setPower(0.7);

                frontLeftMotor.setTargetPosition(moveForward2);
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontLeftMotor.setPower(0.7);

                frontRightMotor.setTargetPosition(moveForward2);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setPower(0.7);

                backRightMotor.setTargetPosition(moveForward2);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setPower(0.7);

                sleep(3000);

                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(500);

                state++;
                break;
            case 4:  // lift move opposite to touch sensor for the backstage
                    int upper = -500;

                    if (!touchT.getState()){
                        lift.setPower(0);
                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        sleep(500);
                        state++;
                    }
                    else {
                        lift.setTargetPosition(upper);  //move forward to the target zone
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(0.7);
                        //sleep(1000);
                    }
                    break;
            case 5:  //move sideways to the right toward the backdrop
                    int movesidewaystoright = 1700;

                    backLeftMotor.setTargetPosition(-movesidewaystoright);  //move forward to the target zone
                    backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backLeftMotor.setPower(0.7);

                    frontLeftMotor.setTargetPosition(movesidewaystoright);
                    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontLeftMotor.setPower(0.7);

                    frontRightMotor.setTargetPosition(-movesidewaystoright);
                    frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontRightMotor.setPower(0.7);

                    backRightMotor.setTargetPosition(movesidewaystoright);
                    backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backRightMotor.setPower(0.7);

                    sleep(1500);

                    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    sleep(500);

                    state++;
                    break;
                case 6:  //move sideways to the right toward the backdrop
                    int adjusting_to_backdrop = 200;

                    backLeftMotor.setTargetPosition(-adjusting_to_backdrop);  //move forward to the target zone
                    backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backLeftMotor.setPower(0.7);

                    frontLeftMotor.setTargetPosition(-adjusting_to_backdrop);
                    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontLeftMotor.setPower(0.7);

                    frontRightMotor.setTargetPosition(adjusting_to_backdrop);
                    frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontRightMotor.setPower(0.7);

                    backRightMotor.setTargetPosition(adjusting_to_backdrop);
                    backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backRightMotor.setPower(0.7);

                    sleep(1200);

                    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    sleep(500);

                    state++;
                    break;
                case 7:  //move sideways to the right toward the backdrop
                    int move_to_backdrop = 800;

                    backLeftMotor.setTargetPosition(move_to_backdrop);  //move forward to the target zone
                    backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backLeftMotor.setPower(0.7);

                    frontLeftMotor.setTargetPosition(move_to_backdrop);
                    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontLeftMotor.setPower(0.7);

                    frontRightMotor.setTargetPosition(move_to_backdrop);
                    frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontRightMotor.setPower(0.7);

                    backRightMotor.setTargetPosition(move_to_backdrop);
                    backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backRightMotor.setPower(0.7);

                    sleep(1200);

                    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    sleep(1000);
                    Lflip.setPosition(0.50);
                    Rflip.setPosition(0.50);
                    sleep(500);
                    state++;
                    break;
                case 8:  //move sideways to the right toward the backdrop
                    int move_away_backdrop = 600;

                    backLeftMotor.setTargetPosition(-move_away_backdrop);  //move forward to the target zone
                    backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backLeftMotor.setPower(0.7);

                    frontLeftMotor.setTargetPosition(-move_away_backdrop);
                    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontLeftMotor.setPower(0.7);

                    frontRightMotor.setTargetPosition(-move_away_backdrop);
                    frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontRightMotor.setPower(0.7);

                    backRightMotor.setTargetPosition(-move_away_backdrop);
                    backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backRightMotor.setPower(0.7);

                    sleep(1000);

                    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    sleep(500);
                    //Lflip.setPosition(0.50);
                    //Rflip.setPosition(0.50);
                    sleep(500);
                    state++;
                    break;

                case 9:  //move sideways to the left away from the backdrop
                    int lower1 = 500;

                    if (!touchB.getState()){
                        lift.setPower(0);
                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        sleep(1000);
                        //state++;
                    }
                    else {
                        lift.setTargetPosition(lower1);  //move forward to the target zone
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(0.7);
                        rotate.setPosition(0.21);
                        //sleep(1000);
                    }
                    int move_sideways_left = 600;

                    backLeftMotor.setTargetPosition(move_sideways_left);  //move forward to the target zone
                    backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backLeftMotor.setPower(0.7);

                    frontLeftMotor.setTargetPosition(-move_sideways_left);
                    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontLeftMotor.setPower(0.7);

                    frontRightMotor.setTargetPosition(move_sideways_left);
                    frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontRightMotor.setPower(0.7);

                    backRightMotor.setTargetPosition(move_sideways_left);
                    backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backRightMotor.setPower(0.7);

                    sleep(1500);

                    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    sleep(500);
                    //Lflip.setPosition(0.50);
                    //Rflip.setPosition(0.50);
                    sleep(500);
                    state++;
                    break;
            default:
            }
            telemetry.addData("State: --> ", state);
            telemetry.update();
        }
    }
}