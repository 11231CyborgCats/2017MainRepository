package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Wyatt on 10/5/2017.
 */
@TeleOp
public class telop extends LinearOpMode {
    HardwareMap  robot;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lm1 = null;
    private DcMotor lm2 = null;
    private DcMotor rm1 = null;
    private DcMotor rm2 = null;
    private DcMotor flicker = null;
    private DcMotor sweeper = null;
    double gyro;
    double IF = .25;
    double IF1 = 0;
    double IF2 = -0.25;
    static final double INCREMENTS = 1;     // amount to slew servo each CYCLE_MS cycle ... increments
    static final int   CYCLE_MS  =  700;     // period of each cycle_ms
    static final double MIN  =  1.0;     // Maximum rotational position
    static final double MAX  =  0.0;     // Minimum rotational position
    Servo servo;

    @Override
    public void runOpMode()  {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        servo = hardwareMap.get(Servo.class, "servo");
        lm1  = hardwareMap.get(DcMotor.class, "lm1");
        lm2 = hardwareMap.get(DcMotor.class, "lm2");
        rm2 = hardwareMap.get(DcMotor.class, "rm2");
        rm1 = hardwareMap.get(DcMotor.class, "rm1");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lm1.setDirection(DcMotor.Direction.FORWARD);
        lm2.setDirection(DcMotor.Direction.FORWARD);
        rm2.setDirection(DcMotor.Direction.FORWARD);
        rm1.setDirection(DcMotor.Direction.FORWARD);

        //double  position = (MAX - MIN) / 2; // Start at halfway position
        //boolean rampUp = true;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_y;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            double POS = servo.getPosition();
            if (POS >= .5){
                servo.setPosition(-.5);
            }
            else  if (POS <= -.5){
                servo.setPosition(.5);
            }
            if (gamepad2.b) {
                servo.setPosition(.5);
            }
            if (gamepad1.dpad_left){
                lm1.setPower(IF1);
                lm2.setPower(IF1);
                rm1.setPower(IF);
                rm1.setPower(IF);
            }
            if (gamepad1.dpad_right){
                lm1.setPower(IF);
                lm2.setPower(IF);
                rm1.setPower(IF1);
                rm2.setPower(IF1);
            }
            if (gamepad1.dpad_up){
                lm1.setPower(IF);
                lm2.setPower(IF);
                rm1.setPower(IF);
                rm2.setPower(IF);
            }
            if (gamepad1.dpad_down){
                lm1.setPower(IF2);
                lm1.setPower(IF2);
                rm1.setPower(IF2);
                rm2.setPower(IF2);
            }
            if (gamepad1.y){
                lm1.setDirection(DcMotorSimple.Direction.REVERSE);
                lm2.setDirection(DcMotorSimple.Direction.REVERSE);
                rm1.setDirection(DcMotorSimple.Direction.REVERSE);
                rm2.setDirection(DcMotorSimple.Direction.REVERSE);

            }
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            lm1.setPower(leftPower);
            lm2.setPower(leftPower);
            rm1.setPower(rightPower);
            rm2.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }


}

