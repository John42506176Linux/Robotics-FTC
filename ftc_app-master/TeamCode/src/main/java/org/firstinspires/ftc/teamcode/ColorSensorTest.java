/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Color Sensor Test", group = "Sensor")
@Disabled
public class ColorSensorTest extends LinearOpMode {

  public ColorSensor colorSensor;    // Hardware Device Object
  public DcMotor  button                    = null;
  private ElapsedTime runtime = new ElapsedTime();

  static final double     BUTTON_SPEED            = 0.1;


  @Override
  public void runOpMode() {

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bPrevState = false;
    boolean bCurrState = false;

    // bLedOn represents the state of the LED.
    boolean bLedOn = true;

    // get a reference to our ColorSensor object.
    colorSensor = hardwareMap.colorSensor.get("color sensor");

    button = hardwareMap.dcMotor.get("button");

    button.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // Set the LED in the beginning
    colorSensor.enableLed(bLedOn);

    // wait for the start button to be pressed.
    waitForStart();

    // while the op mode is active, loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      // check the status of the x button on either gamepad.
      bCurrState = gamepad1.x;

      // check for button state transitions.
      if ((bCurrState == true) && (bCurrState != bPrevState)) {

        // button is transitioning to a pressed state. So Toggle LED
        bLedOn = !bLedOn;
        colorSensor.enableLed(bLedOn);
      }

      // update previous state variable.
      bPrevState = bCurrState;

      // convert the RGB values to HSV values.
      Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

      if (colorSensor.red() > 4) {
        bLedOn = !bLedOn;
        colorSensor.enableLed(bLedOn);
        encoderButton(BUTTON_SPEED, 6, 1.0);
        sleep(50);
        encoderButton(BUTTON_SPEED, -6, 1.0);
      }

      // send the info back to driver station using telemetry function.
      telemetry.addData("LED", bLedOn ? "On" : "Off");
      telemetry.addData("Clear", colorSensor.alpha());
      telemetry.addData("Red  ", colorSensor.red());
      telemetry.addData("Green", colorSensor.green());
      telemetry.addData("Blue ", colorSensor.blue());
      telemetry.addData("Hue", hsvValues[0]);

      // change the background color to match the color detected by the RGB sensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        }
      });

      telemetry.update();
    }
  }
  public void encoderButton(double speed,
                            double inches,
                            double timeoutS)

  {
    int newTarget;

    // Ensure that the opmode is still active
    if (opModeIsActive()) {

      // Determine new target position, and pass to motor controller
      newTarget = button.getCurrentPosition() + (int) (inches * BUTTON_SPEED);
      button.setTargetPosition(newTarget);

      // Turn On RUN_TO_POSITION
      button.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // reset the timeout time and start motion.
      runtime.reset();
      button.setPower(Math.abs(speed));

      // keep looping while we are still active, and there is time left, and both motors are running.
      while (opModeIsActive() &&
              (runtime.seconds() < timeoutS) &&
              (button.isBusy())) {

        // Display it for the driver.
        telemetry.addData("Rack and Pinion", "Running to %7d :%7d", newTarget);
        telemetry.addData("Rack and Pinion 2", "Running at %7d :%7d",
                button.getCurrentPosition());
        telemetry.update();
      }

      // Stop all motion;
      button.setPower(0);

      // Turn off RUN_TO_POSITION
      button.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      //  sleep(250);   // optional pause after each move
    }


  }

}
