package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive By Time", group="Pushbot")
public class auto_1 extends LinearOpMode {

    /* Declare OpMode members. */
    hardware robot   = new hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;

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
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        /*-----------------------------------------
        // Step 1:  Drive forward for 3 seconds
        robot.lm1.setPower(FORWARD_SPEED);
        robot.lm2.setPower(FORWARD_SPEED);
        robot.rm1.setPower(FORWARD_SPEED);
        robot.rm2.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
        robot.lm1.setPower(TURN_SPEED);
        robot.lm2.setPower(TURN_SPEED);
        robot.rm1.setPower(-TURN_SPEED);
        robot.rm2.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backwards for 1 Second
        robot.lm1.setPower(-FORWARD_SPEED);
        robot.lm2.setPower(-FORWARD_SPEED);
        robot.rm1.setPower(-FORWARD_SPEED);
        robot.rm2.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.lm1.setPower(0);
        robot.lm2.setPower(0);
        robot.rm1.setPower(0);
        robot.rm2.setPower(0);
        robot.servo.setPosition(1.0);
*/
        drive(5,5,.5);
        drive(5,0,.5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
    public void drive (float leftDrive,float rightDrive, double speed){
        int NewLeftTarget;
        int NewLeftTarget1;
        int NewRightTarget1;
        int NewRightTarget;
        int moveCounts1;
        int moveCounts;
        int leftSpeed;
        int rightSpeed;

        if (opModeIsActive()) {
            moveCounts = (int)(leftDrive * COUNTS_PER_INCH);
            moveCounts1 = (int)(rightDrive * COUNTS_PER_INCH);

            NewLeftTarget = robot.lm1.getCurrentPosition() + moveCounts;
            NewLeftTarget1 = robot.lm1.getCurrentPosition() + moveCounts;
            NewRightTarget1 = robot.lm1.getCurrentPosition() + moveCounts1;
            NewRightTarget = robot.lm1.getCurrentPosition() + moveCounts1;

            robot.lm1.setPower(speed);
            robot.lm2.setPower(speed);
            robot.rm1.setPower(speed);
            robot.rm2.setPower(speed);


            robot.lm1.setTargetPosition(NewLeftTarget);
            robot.lm2.setTargetPosition(NewLeftTarget1);
            robot.rm1.setTargetPosition(NewRightTarget);
            robot.rm2.setTargetPosition(NewRightTarget1);

            sleep(3000);
        }

    }
    public void driveTurn (int degree,int speed){
        int NewLeftTarget;
        int NewLeftTarget1;
        int NewRightTarget1;
        int NewRightTarget;
        int moveCounts1;
        int moveCounts;
        int leftSpeed;
        int rightSpeed;

    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer = 1;
        boolean  onTarget = false ;
        double leftSpeed;
        double leftSpeed1;
        double rightSpeed;
        double rightSpeed1;

        rightSpeed  = speed * steer;
        rightSpeed1 = speed * steer;
        leftSpeed   = -rightSpeed;
        leftSpeed1 = -rightSpeed1;


        // Send desired speeds to motors.
        robot.lm1.setPower(leftSpeed);
        robot.lm2.setPower(leftSpeed1);
        robot.rm1.setPower(rightSpeed);
        robot.rm2.setPower(rightSpeed1);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f",steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

/*
    public void Drive ( double speed,
                            double distance) {

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
                leftSpeed = speed;
                leftSpeed1 = speed;
                rightSpeed = speed;
                rightSpeed1 = speed;

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
    */
}
