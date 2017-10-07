package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbotCustom
{
    /* Public OpMode members. */
    public DcMotor  leftBackMotor = null;
    public DcMotor  rightBackMotor            = null;
    public DcMotor  leftFrontMotor            = null;
    public DcMotor  rightFrontMotor           = null;
    public DcMotor  leftProjectileMotor       = null;
    public DcMotor  rightProjectileMotor      = null;
    public DcMotor  sweeperMotor              = null;
    public DcMotor  button                    = null;
    public Servo    razor                     = null;

    public static final double MID_SERVO             =  0.4 ;
    public static final double PROJECTILE_ON_POWER   =  1.0 ;
    public static final double PROJECTILE_OFF_POWER  =  0.0 ;
    public static final double SWEEPER_FORWARD_POWER =  1.0 ;
    public static final double SWEEPER_REVERSE_POWER = -1.0 ;
    public static final double BUTTON_FORWARD_POWER  =  1.0 ;
    public static final double BUTTON_REVERSE_POWER  = -1.0 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbotCustom(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftBackMotor        = hwMap.dcMotor.get("left back drive");
        rightBackMotor       = hwMap.dcMotor.get("right back drive");
        leftFrontMotor       = hwMap.dcMotor.get("left front drive");
        rightFrontMotor      = hwMap.dcMotor.get("right front drive");
        leftProjectileMotor  = hwMap.dcMotor.get("left projectile");
        rightProjectileMotor = hwMap.dcMotor.get("right projectile");
        sweeperMotor         = hwMap.dcMotor.get("sweeper");
        button               = hwMap.dcMotor.get("button");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftProjectileMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightProjectileMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        sweeperMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        button.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftProjectileMotor.setPower(0);
        rightProjectileMotor.setPower(0);
        sweeperMotor.setPower(0);
        button.setPower(0);

        // Set all motors to run with or without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftProjectileMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightProjectileMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        button.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        razor = hwMap.servo.get("razor");
        razor.setPosition(MID_SERVO);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of' wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}