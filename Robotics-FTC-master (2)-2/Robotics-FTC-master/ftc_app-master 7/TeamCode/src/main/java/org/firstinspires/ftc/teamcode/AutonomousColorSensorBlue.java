package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.VuMarkTargetResult;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.VuMarkTargetResult;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by ProUser on 12/7/17.
 */
@Autonomous(name = "Autononomous ColorSensorBlue", group = "Pushbot")
@Disabled

public class AutonomousColorSensorBlue extends LinearOpMode
{



        // Use a Pushbot's hardware

        OpenGLMatrix lastLocation = null;
        static final double COUNTS_PER_MOTOR_REV = 1653;    // eg: Ou this is adjusted number for our robot 53.3 ticks per inch
        static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP
        static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double COUNTS_PER_MOTOR_STRAFE_REV = 2204;    // eg: Ou this is adjusted number for our robot 53.3 ticks per inch
        static final double COUNTS_PER_STRAFE_INCH = (COUNTS_PER_MOTOR_STRAFE_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double DRIVE_SPEED = 0.3;
        static final double TURN_SPEED = 0.5;
        final double            CUP_RESET = 0.4 ;

        HardwarePushbotCustom20178 robot = new HardwarePushbotCustom20178();
        private ElapsedTime runtime = new ElapsedTime();
        double f = 0.2;
        double b = -0.2;
        ColorSensor colorSensor;
        VuforiaLocalizer vuforia;
        @Override
        public void runOpMode()
        {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

            robot.init(hardwareMap);
            float hsvValues[] = {0F, 0F, 0F};

            // values is a reference to the hsvValues array.
            final float values[] = hsvValues;

            // get a reference to the RelativeLayout so we can change the background
            // color of the Robot Controller app to match the hue detected by the RGB sensor.
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

            // bPrevState and bCurrState represent the previous and current state of the button.
            boolean bPrevState = false;
            boolean bCurrState = false;

            // bLedOn represents the state of the LED.
            boolean bLedOn = true;

            // get a reference to our ColorSensor object.
            colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
//            robot.leftArmHandle.setPosition(.45);
//            robot.rightArmHandle.setPosition(.55);
            // Set the LED in the beginning
            colorSensor.enableLed(bLedOn);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();

            robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();

            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Send teleemetry message to indicate successful Encoder reset
            telemetry.addData("Path0", "Starting at %7d :%7d",
                    robot.leftFrontMotor.getCurrentPosition(),
                    robot.rightFrontMotor.getCurrentPosition(),
                    robot.leftBackMotor.getCurrentPosition(),
                    robot.rightBackMotor.getCurrentPosition());
            telemetry.update();


            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            if (opModeIsActive()) {


                // check the status of the x button on either gamepad.
                bCurrState = gamepad1.x;

                // check for button state transitions.
                if (bCurrState && (bCurrState != bPrevState)) {

                    // button is transitioning to a pressed state. So Toggle LED
                    bLedOn = !bLedOn;
                    colorSensor.enableLed(bLedOn);
                }

                // update previous state variable.
                bPrevState = bCurrState;
                robot.colorServo.setPosition(0.39);

                // convert the RGB values to HSV values.
                Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

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
                        relativeLayout.setBackgroundColor(Color.HSVToColor(0xFF, values));

                    }
                });
                telemetry.update();
                sleep(500);
                if (colorSensor.red() >= 3)//&& colorSensor.blue() < 2 && colorSensor.green() < 5
                {
                    encoderDrive(DRIVE_SPEED, -3, -3, 20);
//                    robot.colorServo.setPosition(0.0);
                    sleep(1000);
//                    encoderDrive(DRIVE_SPEED, -14, -14, 20);
//                    sleep(500);

                }
                else {
                    encoderDrive(DRIVE_SPEED, 3, 3, 20);
//                    robot.colorServo.setPosition(0.0);

                    sleep(1000);
                }
                sleep(1000);


            }
            //encoderIndDrive(DRIVE_SPEED,20,-20,20,-20,20);
            //encoderDrive(DRIVE_SPEED,3,1,20);
        /*telemetry.addData("Path", "Complete");
        telemetry.update();*/
        }

        public void encoderDrive(double speed,
                                 double leftInches, double rightInches,
                                 double timeoutS) {
            int newLeftFrontTarget;
            int newRightFrontTarget;
            int newLeftBackTarget;
            int newRightBackTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
                robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
                robot.rightBackMotor.setTargetPosition(newRightBackTarget);

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
                        (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.rightBackMotor.isBusy() && robot.leftBackMotor.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
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
        public void encoderIndDrive(double speed,
                                    double leftBackInches,double leftFrontInches,  double rightFrontInches,
                                    double rightBackInches,
                                    double timeoutS) {
            int newLeftFrontTarget;
            int newRightFrontTarget;
            int newLeftBackTarget;
            int newRightBackTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_STRAFE_INCH);
                newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_STRAFE_INCH);
                newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_STRAFE_INCH);
                newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_STRAFE_INCH);
                robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
                robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
                robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
                robot.rightBackMotor.setTargetPosition(newRightBackTarget);

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
                        (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.rightBackMotor.isBusy() && robot.leftBackMotor.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
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
        String format(OpenGLMatrix transformationMatrix) {
            return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
        }
    }


