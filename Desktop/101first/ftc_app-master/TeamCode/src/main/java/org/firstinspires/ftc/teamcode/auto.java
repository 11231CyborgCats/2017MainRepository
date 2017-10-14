package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Wyatt on 10/8/2017.
 */
@Autonomous(name="Auto Drive By Gyro", group="Pushbot")
public class auto extends LinearOpMode {

    /* Declare OpMode members. */
    hardware robot   = new hardware();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device
    ModernRoboticsI2cRangeSensor rangeSensor;

    //doubles//
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.8;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() throws InterruptedException {

                /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.lm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }



        gyro.resetZAxisIntegrator();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        //gyroDrive(DRIVE_SPEED, 48.0, 0.0);    // Drive FWD 48 inches
        //gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        //gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        //gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
        //gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
        //gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        //gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        //gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        //gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches
        gyroDrive(DRIVE_SPEED, 10, 0);
        gyroTurn(TURN_SPEED, 90);
        gyroHold(TURN_SPEED, 90,1);
        gyroDrive(DRIVE_SPEED, 20, 90);
        gyroTurn(TURN_SPEED, 180);
        gyroHold(TURN_SPEED, 180, 1);
        gyroDrive(DRIVE_SPEED, 10, 0);
        gyroTurn(TURN_SPEED, 270);
        gyroHold(TURN_SPEED, 270, 1);
        gyroDrive(DRIVE_SPEED, 10, 0);
        gyroTurn(TURN_SPEED,360);
        gyroHold(TURN_SPEED,360,1);
        gyroDrive(DRIVE_SPEED, 10, 0);

        range_drive(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void range_drive (long delay){
        for ( double distance = rangeSensor.getDistance(DistanceUnit.INCH);distance <=5;){
            gyroDrive(DRIVE_SPEED, 1, 0);
            telemetry.addData("Moving", distance);
            sleep(delay);
        }
            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
    }
    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     newLeftTarget1;
        int     newRightTarget1;
        int     moveCounts;
        double  max;
        double  max1;
        double  error;
        double  steer;
        double  leftSpeed1;
        double  rightSpeed1;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.lm1.getCurrentPosition() + moveCounts;
            newLeftTarget1 = robot.lm2.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rm1.getCurrentPosition() + moveCounts;
            newRightTarget1 = robot.rm2.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.lm1.setTargetPosition(newLeftTarget);
            robot.lm2.setTargetPosition(newLeftTarget1);
            robot.rm1.setTargetPosition(newRightTarget);
            robot.rm2.setTargetPosition(newRightTarget1);

            robot.lm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.lm1.setPower(speed);
            robot.lm2.setPower(speed);
            robot.rm1.setPower(speed);
            robot.rm2.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.lm1.isBusy() && robot.lm2.isBusy() && robot.rm1.isBusy() && robot.rm2.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                leftSpeed1 = speed - steer;
                rightSpeed = speed + steer;
                rightSpeed1 = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(leftSpeed1));
                max1 = Math.max(Math.abs(rightSpeed), Math.abs(rightSpeed1));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    leftSpeed1 /= max;
                    rightSpeed /= max1;
                    rightSpeed1 /= max1;
                }

                robot.lm1.setPower(leftSpeed);
                robot.lm2.setPower(leftSpeed1);
                robot.rm1.setPower(rightSpeed);
                robot.rm2.setPower(rightSpeed1);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.lm1.getCurrentPosition(),
                        robot.rm1.getCurrentPosition(), robot.lm2.getCurrentPosition(), robot.rm2.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed,leftSpeed1,rightSpeed1);
                telemetry.update();
            }

            // Stop all motion;
            robot.lm1.setPower(0);
            robot.lm2.setPower(0);
            robot.rm1.setPower(0);
            robot.rm2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.lm1.setPower(0);
        robot.lm2.setPower(0);
        robot.rm1.setPower(0);
        robot.rm2.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double leftSpeed1;
        double rightSpeed;
        double rightSpeed1;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            leftSpeed1 = 0.0;
            rightSpeed = 0.0;
            rightSpeed1 = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            rightSpeed1 = speed * steer;
            leftSpeed   = -rightSpeed;
            leftSpeed1 = -rightSpeed1;
        }

        // Send desired speeds to motors.
        robot.lm1.setPower(leftSpeed);
        robot.lm2.setPower(leftSpeed1);
        robot.rm1.setPower(rightSpeed);
        robot.rm2.setPower(rightSpeed1);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}

