package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.Teleop;

public class TurtleRobot {
    /* Public OpMode members. */
    public  DcMotor rightFront = null;
    public  DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public  DcMotor leftBack = null;
    public DcMotor leftSlide = null;
    public DcMotor rightSlide = null;
    public DcMotor rightActuator = null;
    public DcMotor leftActuator = null;
    public ElapsedTime runtime = new ElapsedTime();
    public OpMode myOpMode = null;
    public Servo leftHorizontalSlide = null;
    public Servo rightHorizontalSlide = null;
    public Servo intake = null;
    public Servo outtake = null;
    public Servo topLeft = null;
    public Servo bottomLeft = null;
   // public Servo topRight = null;
    public Servo bottomRight = null;
    public Servo smartServo = null;
    public Servo arm = null;
    public RevBlinkinLedDriver light = null;
    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public TurtleRobot(OpMode opmode) { myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        /**
         * Control Hub:
         * Motors:
             * 0 - rightSlide
             * 1 - leftSlide
             * 2 - leftFront
             * 3 - rightFront
         * Servos:
             * 0 - top_servo
             * 1 - bottom_left
             * 2 - bottom_right
             * 3 - right_horizontal_slide
             * 4 - left_horizontal_slide
             * 5 - intake_claw
         * Expansion Hub:
         * Motors:
             * 0 - rightBack
             * 1 - leftBack
             * 2 - right_actuator
             * 3 - left_actuator
         * Servos:
             * 1 - smart_servo
             * 3 - outake_claw
             * 5 - arm_servo
        **/

        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        rightBack = hwMap.get(DcMotorEx.class, "rightBack");
        leftSlide = hwMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hwMap.get(DcMotorEx.class, "rightSlide");
        rightActuator = hwMap.get(DcMotorEx.class, "right_actuator");
        leftActuator = hwMap.get(DcMotorEx.class, "left_actuator");
        leftHorizontalSlide = hwMap.get(Servo.class, "left_horizontal_slide");
        rightHorizontalSlide = hwMap.get(Servo.class, "right_horizontal_slide");

        light = hwMap.get(RevBlinkinLedDriver.class, "light");
        intake = hwMap.get(Servo.class, "intake_claw");
        outtake = hwMap.get(Servo.class, "outake_claw");
        topLeft = hwMap.get(Servo.class, "top_servo");
//        topRight = hwMap.get(Servo.class, "top_right");
        bottomLeft = hwMap.get(Servo.class, "bottom_left");
        bottomRight = hwMap.get(Servo.class, "bottom_right");
        smartServo = hwMap.get(Servo.class, "smart_servo");
        arm = hwMap.get(Servo.class, "arm_servo");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftSlide.setDirection(DcMotorEx.Direction.FORWARD);
        rightSlide.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightActuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftSlide.setPower(0);
        rightSlide.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
    }
}