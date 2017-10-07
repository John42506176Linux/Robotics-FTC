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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbotCustom class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test TeleOp", group="Robotics")
//@Disabled
public class POVCustom extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbotCustom robot           = new HardwarePushbotCustom();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double          cupOffset      = 0;                       // Servo mid position
    final double    CUP_SPEED      = 0.25;                    // sets rate to move servo
    double          driveSpeed     = 0.35;

    @Override
    public void runOpMode() {
        double r;
        double robotAngle;
        double rightX;

        // double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // ColorSensor colorSensor = hardwareMap.colorSensor.get("sensor_color");
        // colorSensor.enableLed(false);
        // OpticalDistanceSensor lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");
        // lightSensor.enableLed(false);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                rightX = gamepad1.right_stick_x;

                mecanumDrive(r, robotAngle, rightX);

            }*/

            if ((Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) && gamepad1.left_stick_y > 0)
            {
                drive(1.0, 1.0, 1.0, 1.0);
            }

            else if ((Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) && gamepad1.left_stick_y < 0)
            {
                drive(-1.0, -1.0, -1.0, -1.0);
            }

            else if ((Math.abs(gamepad1.left_stick_y) < Math.abs(gamepad1.left_stick_x)) && gamepad1.left_stick_x > 0)
            {
                drive(1.0, -1.0, -1.0, 1.0);
            }

            else if ((Math.abs(gamepad1.left_stick_y) < Math.abs(gamepad1.left_stick_x)) && gamepad1.left_stick_x < 0)
            {
                drive(-1.0, 1.0, 1.0, -1.0);
            }

            else if (gamepad1.right_stick_x != 0) {
                robot.leftBackMotor.setPower(-gamepad1.right_stick_x);
                robot.rightBackMotor.setPower(gamepad1.right_stick_x);
                robot.leftFrontMotor.setPower(-gamepad1.right_stick_x);
                robot.rightFrontMotor.setPower(gamepad1.right_stick_x);
            } else if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
                robot.leftBackMotor.setPower(0);
                robot.rightBackMotor.setPower(0);
                robot.leftFrontMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);
            }

            if (gamepad2.b)

                robot.razor.setPosition(CUP_SPEED);
            else if (gamepad2.x)
                robot.razor.setPosition(-1.0 * CUP_SPEED);

            // Move both servos to new position. Assume servos are mirror image of each other.
            // cupOffset = Range.clip(cupOffset, -1.5, 1.5);
            //robot.razor.setPosition(robot.MID_SERVO - cupOffset);


            // Use gamepad buttons to turn projectile launchers on (X) and off (B)
            if (gamepad1.right_bumper) {
                robot.leftProjectileMotor.setPower(robot.PROJECTILE_ON_POWER);
                robot.rightProjectileMotor.setPower(robot.PROJECTILE_ON_POWER);
            } else if (gamepad1.left_bumper) {
                robot.leftProjectileMotor.setPower(robot.PROJECTILE_OFF_POWER);
                robot.rightProjectileMotor.setPower(robot.PROJECTILE_OFF_POWER);
            } else {
                robot.leftProjectileMotor.setPower(0.0);
                robot.rightProjectileMotor.setPower(0.0);
            }

            // Use gamepad buttons to turn sweeper motor on (Y) and off (A)
            if (gamepad2.y) {
                robot.sweeperMotor.setPower(robot.SWEEPER_FORWARD_POWER);
            } else if (gamepad2.a) {
                robot.sweeperMotor.setPower(robot.SWEEPER_REVERSE_POWER);
            } else {
                robot.sweeperMotor.setPower(0.0);
            }
            if (gamepad2.right_bumper) {
                robot.button.setPower(robot.BUTTON_FORWARD_POWER);

            } else if (gamepad2.left_bumper) {
                robot.button.setPower(robot.BUTTON_REVERSE_POWER);

                
            } else {
                robot.button.setPower(0.0);

            }


            // Send telemetry message to signify robot running;
            // telemetry.addData("ramp",  "Offset = %.2f", rampOffset);
            // telemetry.addData("left",  "%.2f", left);
            // telemetry.addData("right", "%.2f", right);
            // telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);

            idle();
        }
    }

    public void drive(double lb, double rb, double lf, double rf)
    {
        robot.leftBackMotor.setPower(lb * driveSpeed);
        robot.rightBackMotor.setPower(rb * driveSpeed);
        robot.leftFrontMotor.setPower(lf * driveSpeed);
        robot.rightFrontMotor.setPower(rf * driveSpeed);
    }

    /*
    public void mecanumDrive(double radius, double angle, double xval)
    {

        final double lb = (radius * Math.cos(angle) + xval)/2;
        final double rb = (radius * Math.sin(angle) - xval)/2;
        final double lf = (radius * Math.sin(angle) + xval)/2;
        final double rf = (radius * Math.cos(angle) - xval)/2;

        robot.leftBackMotor.setPower(lb);
        robot.rightBackMotor.setPower(rb);
        robot.leftFrontMotor.setPower(lf);
        robot.rightFrontMotor.setPower(rf);

    }*/
}
