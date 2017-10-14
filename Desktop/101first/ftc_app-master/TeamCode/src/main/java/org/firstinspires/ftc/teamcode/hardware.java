package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Wyatt on 10/8/2017.
 */

public class hardware {
    /* Public OpMode members. */
    public DcMotor lm1  = null;
    public DcMotor lm2  = null;
    public DcMotor rm1 = null;
    public DcMotor rm2 = null;
    public Servo servo = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lm1  = hwMap.get(DcMotor.class, "lm1");
        lm2 = hwMap.get(DcMotor.class, "lm2");
        lm1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        lm2.setDirection(DcMotor.Direction.FORWARD);
        rm1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rm2.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        lm1.setPower(0);
        lm2.setPower(0);
        rm1.setPower(0);
        rm2.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        lm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Define and initialize ALL installed servos.
        servo  = hwMap.get(Servo.class, "servo");
        servo.setPosition(MID_SERVO);
    }
}
