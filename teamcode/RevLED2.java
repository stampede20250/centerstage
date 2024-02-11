package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.initialization.qual.Initialized;

@TeleOp(name="RevLED2")
//@Disabled

public class RevLED2 extends OpMode {

    RevBlinkinLedDriver blinkinLedDriver;

    int temp = 1;

    public void init() {
        // Rev Blinkin Init
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN;
        blinkinLedDriver.setPattern(pattern);
        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        // starts at BLACK
        // runs in DARK_GREEN
        // turns DARK_RED on time
    }

    public void loop() {
        if (temp == 1) {
            temp = 2;
        }
        if (time >= 110 && time <125) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        } else if (time >= 90 && time < 110) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (time >= 85 && time < 90) { //remove if issue
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD); //remove if issue
                } else {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

                    //display elaspsed time on telemetry
                    telemetry.addData("Status", "Runtime:  " + getRuntime());
                    telemetry.update();

                }
            }
        }

