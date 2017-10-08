package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbotCustom class.
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "pleasewrk", group = "Linear Opmode")


public class POVCustom120178 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbotCustom20178 robot           = new HardwarePushbotCustom20178();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double cupOffset = 0;                       // Servo mid position
    final double ARM_SPEED = 0.03; // sets rate to move servo
    double ARM_POSITION_LEFT = 0.05;
    double ARM_POSITION_RIGHT = 0.95;
    final double initialleft = 0.05;
    final double initialright = 0.95;
    double driveSpeed = 0.35;
    final double HOLD = -0.05;
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

            if ((Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) && gamepad1.left_stick_y > 0) {
                drive(1.0, 1.0, 1.0, 1.0);
            } else if ((Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) && gamepad1.left_stick_y < 0) {
                drive(-1.0, -1.0, -1.0, -1.0);
            } else if ((Math.abs(gamepad1.left_stick_y) < Math.abs(gamepad1.left_stick_x)) && gamepad1.left_stick_x > 0) {
                drive(1.0, -1.0, -1.0, 1.0);
            } else if ((Math.abs(gamepad1.left_stick_y) < Math.abs(gamepad1.left_stick_x)) && gamepad1.left_stick_x < 0) {
                drive(-1.0, 1.0, 1.0, -1.0);
            } else if (gamepad1.right_stick_x != 0) {
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

            if (gamepad2.x) {

                robot.rightArmHandle.setPosition(ARM_POSITION_RIGHT);
                ARM_POSITION_RIGHT -= ARM_SPEED;
                robot.leftArmHandle.setPosition(ARM_POSITION_LEFT);
                ARM_POSITION_LEFT += ARM_SPEED;
            }
            else if (gamepad2.y) {
                robot.rightArmHandle.setPosition(initialright);
                ARM_POSITION_RIGHT =initialright;
                robot.leftArmHandle.setPosition(initialleft);
                ARM_POSITION_LEFT = initialleft;
            }
//            if (gamepad2.a) {
//
//                robot.leftArmHandle.setPosition(ARM_POSITION_LEFT);
//                ARM_POSITION_LEFT += ARM_SPEED;
//            }
//            else if (gamepad2.y) {
//                robot.leftArmHandle.setPosition(initialleft);
//                ARM_POSITION_LEFT = initialleft;
//            }

            // Move both servos to new position. Assume servos are mirror image of each other.
            // cupOffset = Range.clip(cupOffset, -1.5, 1.5);
            //robot.razor.setPosition(robot.MID_SERVO - cupOffset);


            // Use gamepad buttons to turn sweeper motor on (Y) and off (A)
            if (gamepad2.right_bumper) {
                robot.bottomArmMotor.setPower(robot.ARM_FORWARD_POWER);
            } else if (gamepad2.left_bumper) {
                robot.bottomArmMotor.setPower(robot.ARM_REVERSE_POWER);
            }
            else if (gamepad2.dpad_left) {
                robot.bottomArmMotor.setPower(HOLD);
            }
            else {
                robot.bottomArmMotor.setPower(0.0);


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
    }

    public void drive(double lb, double rb, double lf, double rf) {
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
