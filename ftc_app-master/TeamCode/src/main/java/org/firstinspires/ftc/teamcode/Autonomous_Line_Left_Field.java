/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive To Line", group="Pushbot")
@Disabled
public class Autonomous_Line_Left_Field extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbotCustom         robot   = new HardwarePushbotCustom();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
    OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor
    ColorSensor             colorSensor;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     WHITE_THRESHOLD         = 0.2;  // spans between 0.1 - 0.5 from dark to light
    static final double     APPROACH_SPEED          = 0.5;
    static final double     DRIVE_SPEED             = 0.6;

    static final double     COUNTS_PER_MOTOR_REV    = 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     BUTTON_SPEED            = 0.1;
    int color;

    @Override
    public void runOpMode() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);

        // get a reference to our Optical Distance Sensor object.
        lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // get reference to Color Sensor
        colorSensor = hardwareMap.colorSensor.get("color sensor");

        // set state of LED of color sensor
        boolean bLedOn = false;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
        }

        // Start the robot moving forward, and then begin looking for a white line.
        robot.leftFrontMotor.setPower(APPROACH_SPEED);
        robot.rightFrontMotor.setPower(APPROACH_SPEED);
        robot.leftBackMotor.setPower(APPROACH_SPEED);
        robot.rightBackMotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
        }

        // Stop all motors
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightBackMotor.setPower(0);

        //code to press first beacon button
        encoderDrive(DRIVE_SPEED,  6, -6, 1.0);  // rotate until parallel to wall
        encoderDrive(DRIVE_SPEED,  6,  6, 1.0);  // drive until on level with far side of beacon

        bLedOn = !bLedOn;
        colorSensor.enableLed(bLedOn);
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        if (colorSensor.red() > 4)
        {
            robot.button.setPower(.05);
            while (colorSensor.red() < 8)
            {
                // do nothing
            }
            robot.button.setPower(-.05);
            while (colorSensor.red() > 5)
            {
                // do nothing
            }
            robot.button.setPower(0);
        }
        else
        {
            encoderDrive(DRIVE_SPEED,  -12,  -12, 2.0);
            robot.button.setPower(.05);
            while (colorSensor.red() < 8)
            {
                // do nothing
            }
            robot.button.setPower(-.05);
            while (colorSensor.red() > 5)
            {
                // do nothing
            }
            robot.button.setPower(0);
        }
        bLedOn = !bLedOn;
        colorSensor.enableLed(bLedOn);

        encoderDrive(DRIVE_SPEED,  6,  6, 1.0);  // drive until slightly past 1st white line

        //Start the robot moving forward, and then begin looking for a white line.
        robot.leftFrontMotor.setPower(APPROACH_SPEED);
        robot.rightFrontMotor.setPower(APPROACH_SPEED);
        robot.leftBackMotor.setPower(APPROACH_SPEED);
        robot.rightBackMotor.setPower(APPROACH_SPEED);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();
        }

        // Stop all motors
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightBackMotor.setPower(0);

        //code to press second beacon button
        encoderDrive(DRIVE_SPEED,  6,  6, 1.0);  // drive until on level with far side of beacon
        // color sensor test (beacon 2): if correct color, activate rack and pinion. If not, drive back a bit and same thing.
        bLedOn = !bLedOn;
        colorSensor.enableLed(bLedOn);

        if (colorSensor.red() > 4)
        {
            robot.button.setPower(.05);
            while (colorSensor.red() < 8)
            {
                // do nothing
            }
            robot.button.setPower(-.05);
            while (colorSensor.red() > 5)
            {
                // do nothing
            }
            robot.button.setPower(0);
        }
        else
        {
            encoderDrive(DRIVE_SPEED,  -12,  -12, 2.0);
            robot.button.setPower(.05);
            while (colorSensor.red() < 8)
            {
                // do nothing
            }
            robot.button.setPower(-.05);
            while (colorSensor.red() > 5)
            {
                // do nothing
            }
            robot.button.setPower(0);
        }
        bLedOn = !bLedOn;
        colorSensor.enableLed(bLedOn);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);
            robot.leftBackMotor.setTargetPosition(newLeftTarget);
            robot.rightBackMotor.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition(),
                        robot.leftBackMotor.getCurrentPosition(),
                        robot.rightBackMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderButton(double speed,
                             double inches,
                             double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = robot.button.getCurrentPosition() + (int)(inches * BUTTON_SPEED);
            robot.button.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.button.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.button.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.button.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Rack and Pinion",  "Running to %7d :%7d", newTarget);
                telemetry.addData("Rack and Pinion 2",  "Running at %7d :%7d",
                        robot.button.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.button.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.button.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}
