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

@TeleOp(name = "Lebron", group = "Linear Opmode")


public class POVCustom120178 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbotCustom20178 robot = new HardwarePushbotCustom20178();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double cupOffset = 0;                       // Servo mid position
    final double ARM_SPEED = 0.03; // sets rate to move servo
    double ARM_POSITION_LEFT = 0.00;
    double ARM_POSITION_RIGHT = 1.00;
    final double HOLD = -0.2;
    double Forward = -0.40;
    double Backward = 0.40;

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

            /*if(shouldMecanumDrive)
            {
                Mecanum.Motion motion = Mecanum.joystickToMotion(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_y,gamepad1.right_stick_y);
                Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
                robot.leftFrontMotor.setPower(wheels.frontLeft);
                robot.rightFrontMotor.setPower(wheels.frontRight);
                robot.leftBackMotor.setPower(wheels.backLeft);
                robot.rightBackMotor.setPower(wheels.backRight);
            }
            else
            {
                // Use old code in comments, I really hope we don't have to do this though.
            }*/
            /*if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                rightX = gamepad1.right_stick_x;

                mecanumDrive(r, robotAngle, rightX);

            }*/
//
//            if ((Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) && gamepad1.left_stick_y > 0) {
//                drive(1.0, 1.0, 1.0, 1.0);
//            } else if ((Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.left_stick_x)) && gamepad1.left_stick_y < 0) {
//                drive(-1.0, -1.0, -1.0, -1.0);
//            } else if ((Math.abs(gamepad1.left_stick_y) < Math.abs(gamepad1.left_stick_x)) && gamepad1.left_stick_x > 0) {
//                drive(1.0, -1.0, -1.0, 1.0);
//            } else if ((Math.abs(gamepad1.left_stick_y) < Math.abs(gamepad1.left_stick_x)) && gamepad1.left_stick_x < 0) {
//                drive(-1.0, 1.0, 1.0, -1.0);
//            } else if (gamepad1.right_stick_x != 0) {
//                robot.leftBackMotor.setPower(gamepad1.right_stick_x);
//                robot.rightBackMotor.setPower(-gamepad1.right_stick_x);
//                robot.leftFrontMotor.setPower(gamepad1.right_stick_x);
//                robot.rightFrontMotor.setPower(-gamepad1.right_stick_x);
//            } else if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
//                robot.leftBackMotor.setPower(0);
//                robot.rightBackMotor.setPower(0);
//                robot.leftFrontMotor.setPower(0);
//                robot.rightFrontMotor.setPower(0);
//            }
//            if (gamepad1.dpad_up) {
//                drive(1.0, 1.0, 1.0, 1.0);
//            } else if (gamepad1.dpad_down) {
//                drive(-1.0, -1.0, -1.0, -1.0);
//            } else if (gamepad1.dpad_left) {
//                drive(-1.0, 1.0, 1.0, -1.0);
//            } else if (gamepad1.dpad_right) {
//                drive(1.0, -1.0, -1.0, 1.0);
//            } else if (gamepad1.right_stick_x != 0) {
//                robot.leftBackMotor.setPower(gamepad1.right_stick_x);
//                robot.rightBackMotor.setPower(-gamepad1.right_stick_x);
//                robot.leftFrontMotor.setPower(gamepad1.right_stick_x);
//                robot.rightFrontMotor.setPower(-gamepad1.right_stick_x);
//            } else {
//                robot.leftBackMotor.setPower(0);
//                robot.rightBackMotor.setPower(0);
//                robot.leftFrontMotor.setPower(0);
//                robot.rightFrontMotor.setPower(0);
//            }
//            if (gamepad1.dpad_down) {
//                drive(1.0, 1.0, 1.0, 1.0);
//            } else if (gamepad1.dpad_up) {
//                drive(-1.0, -1.0, -1.0, -1.0);
//            } else if (gamepad1.dpad_left) {
//                drive(-1.0, 1.0, 1.0, -1.0);
//            } else if (gamepad1.dpad_right) {
//                drive(1.0, -1.0, -1.0, 1.0);
////            } else if (gamepad1.right_stick_x != 0) {
////                robot.leftBackMotor.setPower(gamepad1.right_stick_x);
////                robot.rightBackMotor.setPower(-gamepad1.right_stick_x);
////                robot.leftFrontMotor.setPower(gamepad1.right_stick_x);
////                robot.rightFrontMotor.setPower(-gamepad1.right_stick_x);
////            } else if (gamepad1.right_stick_x ==0 && gamepad1.right_stick_y == 0 && gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0)
////                robot.leftBackMotor.setPower(0);
////                robot.rightBackMotor.setPower(0);
////                robot.leftFrontMotor.setPower(0);
////                robot.rightFrontMotor.setPower(0);
////            }
//            }
//            else {
//                drive(0,0,0,0);
//            }
            if (gamepad1.dpad_up) {
                robot.leftFrontMotor.setPower(Forward);
                robot.rightFrontMotor.setPower(Forward);
                robot.leftBackMotor.setPower(Forward);
                robot.rightBackMotor.setPower(Forward);
            } else if (gamepad1.dpad_down) {
                robot.leftFrontMotor.setPower(Backward);
                robot.rightFrontMotor.setPower(Backward);
                robot.leftBackMotor.setPower(Backward);
                robot.rightBackMotor.setPower(Backward);
            } else if (gamepad1.dpad_left) {
                robot.leftFrontMotor.setPower(Backward);
                robot.rightFrontMotor.setPower(Forward);
                robot.leftBackMotor.setPower(Forward);
                robot.rightBackMotor.setPower(Backward);
            } else if (gamepad1.dpad_right) {
                robot.leftFrontMotor.setPower(Forward);
                robot.rightFrontMotor.setPower(Backward);
                robot.leftBackMotor.setPower(Backward);
                robot.rightBackMotor.setPower(Forward);
            }
            else if (gamepad1.left_bumper) {
                robot.leftBackMotor.setPower(Backward);
                robot.leftFrontMotor.setPower(Backward);
                robot.rightFrontMotor.setPower(Forward);
                robot.rightBackMotor.setPower(Forward);
            } else if (gamepad1.right_bumper) {
                robot.leftBackMotor.setPower(Forward);
                robot.leftFrontMotor.setPower(Forward);
                robot.rightFrontMotor.setPower(Backward);
                robot.rightBackMotor.setPower(Backward);
            } else {
                robot.leftFrontMotor.setPower(0);
                robot.rightFrontMotor.setPower(0);
                robot.leftBackMotor.setPower(0);
                robot.rightBackMotor.setPower(0);
            }
            if (gamepad2.x) {

                ARM_POSITION_RIGHT -= ARM_SPEED;
                robot.rightArmHandle.setPosition(ARM_POSITION_RIGHT);
                ARM_POSITION_LEFT += ARM_SPEED;
                robot.leftArmHandle.setPosition(ARM_POSITION_LEFT);
            } else if (gamepad2.y) {

                ARM_POSITION_RIGHT += ARM_SPEED;
                robot.rightArmHandle.setPosition(ARM_POSITION_RIGHT);
                ARM_POSITION_LEFT -= ARM_SPEED;
                robot.leftArmHandle.setPosition(ARM_POSITION_LEFT);
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
            if (gamepad2.dpad_up) {
                robot.bottomArmMotor.setPower(robot.RACK_AND_PINON);
            } else if (gamepad2.dpad_down) {
                robot.bottomArmMotor.setPower(robot.RACK_AND_PINON_REVERSE);
            }  else {
                robot.bottomArmMotor.setPower(0.0);
            }
            if (gamepad2.a) {
                robot.leftArmMotor.setPower(robot.ARM_FORWARD_POWER);
                robot.rightArmMotor.setPower(robot.ARM_FORWARD_POWER);
            } else if (gamepad2.b) {
                robot.leftArmMotor.setPower(robot.ARM_REVERSE_POWER);
                robot.rightArmMotor.setPower(robot.ARM_REVERSE_POWER);
            }
            else if (gamepad2.dpad_left) {
                robot.leftArmMotor.setPower(-HOLD);
                robot.rightArmMotor.setPower(-HOLD);
            }
            else if (gamepad2.dpad_right) {
                robot.leftArmMotor.setPower(HOLD);
                robot.rightArmMotor.setPower(HOLD);
            }
            else if (gamepad2.left_bumper) {
                robot.leftArmMotor.setPower(robot.supafowad);
                robot.rightArmMotor.setPower(robot.supafowad);
            }
            else if (gamepad2.right_bumper) {
                robot.leftArmMotor.setPower(robot.supabakwad);
                robot.rightArmMotor.setPower(robot.supabakwad);
            }
            else {
                robot.leftArmMotor.setPower(0.0);
                robot.rightArmMotor.setPower(0.0);


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
}


//    public void drive(double lf, double rf, double lb, double rb) {
//        robot.leftFrontMotor.setPower(lf * driveSpeed);
//        robot.rightFrontMotor.setPower(rf * driveSpeed);
//        robot.leftBackMotor.setPower(lb * driveSpeed);
//        robot.rightBackMotor.setPower(rb * driveSpeed);
//    }

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

