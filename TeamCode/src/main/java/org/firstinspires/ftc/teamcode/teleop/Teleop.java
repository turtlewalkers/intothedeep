package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Config
@TeleOp
public class Teleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    int SLIDE_HEIGHT = 0;
    TurtleRobot robot = new TurtleRobot(this);
    double topPos, bottomPos;
    boolean softlock = true;
    public static double CONSTANT;
    public static double OUTTAKECLAW1 = 0.2;
    public static double OUTTAKECLAW2 = 0.05;
    public static double SMARTSERVO1 = 0.85;
    public static double SMARTSERVO2 = 0.15;
    static double TX_PICKUP_SMARTSERVO = 0.65;
    static double BASKET_SMARTSERVO = 0.45;
    static double BASKET_ARMSERVO = 0.7;
    static double TX_PICKUP_ARMSERVO = 0.1;
    static double TOPINIT = 0;
    public static double OPENINTAKE = 1;
    public static double CLOSEINTAKE = 0.4;
    public static double TOP_OBSERVE = 0.05;
    public static double BOTTOM_OBSERVE = 0.8;
    public static double TOP_TRANSFER = 0.2;
    public static double BOTTOM_TRANSFER = 0.12;
    public static double TOP_PICK = 0.225;
    public static double BOTTOM_PICK = 0.8;
    public static double BOTTOMINIT = 0.1;
    public static double PICKING_UP = 0.28;
    public static double PADLEFT = 0.1;
    public static double TOP_SCAN_SUB = 0.05;
    public static double BOTTOM_SCAN_SUB = 0.5;
    boolean servolock = false;
    static double SPEC_PICK_SMARTSERVO = 0.3;
    static double SPEC_PICK_ARMSERVO = 0.3;
    public static double SLIDE = -1250;
    double BOTTOM_LEFT = BOTTOMINIT;
    double BOTTOM_RIGHT = BOTTOMINIT;
    double HORIZONTALSLIDE;

    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double PULLEY_DIAMETER_INCHES = 1.404;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (PULLEY_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.topLeft.setPosition(TOPINIT);
        robot.topRight.setPosition(TOPINIT);
        robot.bottomRight.setPosition(BOTTOMINIT);
        robot.bottomLeft.setPosition(BOTTOMINIT);
        //robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PICKING_UP = 0.85;
        HORIZONTALSLIDE = 0;
        robot.smartServo.setPosition(0.15);

        waitForStart();

        int linearSlideTargetHeight = 0;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double divisor = 2;
            if (gamepad1.left_trigger != 0) {
                divisor = 5;
            }

            robot.leftFront.setPower((y + x + rx) / divisor);
            robot.leftBack.setPower((y - x + rx) / divisor);
            robot.rightFront.setPower((y - x - rx) / divisor);
            robot.rightBack.setPower((y + x - rx) / divisor);

            HORIZONTALSLIDE -= gamepad2.left_stick_y / 300;
            HORIZONTALSLIDE = Math.min(HORIZONTALSLIDE, 0.9);

            robot.leftHorizontalSlide.setPosition(HORIZONTALSLIDE);
            robot.rightHorizontalSlide.setPosition(HORIZONTALSLIDE);

            if (gamepad2.right_stick_y != 0) {
                robot.leftSlide.setPower(gamepad2.right_stick_y);
                robot.rightSlide.setPower(gamepad2.right_stick_y);
            }
            if (gamepad2.b && servolock == false) { //Picks
                robot.topRight.setPosition(TOP_PICK);
                robot.topLeft.setPosition(TOP_PICK);
                robot.bottomRight.setPosition(BOTTOM_PICK);
                robot.bottomLeft.setPosition(BOTTOM_PICK);
                BOTTOM_LEFT = BOTTOM_PICK;
                BOTTOM_RIGHT = BOTTOM_PICK;
                double multiplier = 0.1;

                if (gamepad2.right_stick_x != 0) {
                    BOTTOM_LEFT += gamepad2.right_stick_x*multiplier;
                    BOTTOM_RIGHT -= gamepad2.right_stick_x*multiplier;
                }
                robot.bottomRight.setPosition(BOTTOM_RIGHT);
                robot.bottomLeft.setPosition(BOTTOM_LEFT);
                sleep(200);
                robot.intake.setPosition(CLOSEINTAKE);
            } else if (gamepad2.y && servolock == false) { // Observes
                robot.topRight.setPosition(TOP_OBSERVE);
                robot.topLeft.setPosition(TOP_OBSERVE);
                robot.intake.setPosition(OPENINTAKE);
                BOTTOM_LEFT = BOTTOM_OBSERVE;
                BOTTOM_RIGHT = BOTTOM_OBSERVE;
                double multiplier = 0.1;

                if (gamepad2.right_stick_x != 0) {
                    BOTTOM_LEFT += gamepad2.right_stick_x*multiplier;
                    BOTTOM_RIGHT -= gamepad2.right_stick_x*multiplier;
                }
                robot.bottomRight.setPosition(BOTTOM_RIGHT);
                robot.bottomLeft.setPosition(BOTTOM_LEFT);

            } else if (gamepad2.x && servolock == false) { //intake transfer
                robot.topRight.setPosition(TOP_TRANSFER);
                robot.topLeft.setPosition(TOP_TRANSFER);
                robot.bottomRight.setPosition(BOTTOM_TRANSFER);
                robot.bottomLeft.setPosition(BOTTOM_TRANSFER);
            } else if (gamepad2.a && servolock == false) { //scans the submersible
                robot.topRight.setPosition(TOP_SCAN_SUB);
                robot.topLeft.setPosition(TOP_SCAN_SUB);
                robot.bottomRight.setPosition(BOTTOM_SCAN_SUB);
                robot.bottomLeft.setPosition(BOTTOM_SCAN_SUB);
                robot.intake.setPosition(OPENINTAKE);
            }

            if (gamepad1.left_trigger > 0.1) {
                servolock = true;
            }
            if (gamepad1.right_trigger > 0.1) {
                servolock = false;
            }

            if (gamepad2.right_bumper) {
                robot.intake.setPosition(OPENINTAKE);
            } else if (gamepad2.left_bumper) {
                robot.intake.setPosition(CLOSEINTAKE);
            }

//            if (gamepad2.dpad_left) {
//                BOTTOM_LEFT = PADLEFT;
//                BOTTOM_RIGHT = PADLEFT;
//            } else if (gamepad2.dpad_down && softlock) {
//                BOTTOM_LEFT = 0.8;
//                BOTTOM_RIGHT = 0.8;
//            }

            if (gamepad1.left_bumper) {
                robot.outtake.setPosition(OUTTAKECLAW1);
            } else if (gamepad1.right_bumper) {
                robot.outtake.setPosition(OUTTAKECLAW2);
            }

            if (gamepad1.dpad_right) {
                robot.smartServo.setPosition((SMARTSERVO1));
            }
            if (gamepad1.dpad_left) {
                robot.smartServo.setPosition(SMARTSERVO2);
            }
            if (gamepad1.dpad_down) { // outtake action - pick up from transfer box
                robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
                robot.arm.setPosition(TX_PICKUP_ARMSERVO);
            }

            if (gamepad1.dpad_up) { // outtake action - drop position for basket
                robot.smartServo.setPosition(BASKET_SMARTSERVO);
                robot.arm.setPosition(BASKET_ARMSERVO);
            }

            if (gamepad1.y) { // outtake action - pick specimen from wall
                robot.smartServo.setPosition(SPEC_PICK_SMARTSERVO);
                robot.arm.setPosition(SPEC_PICK_ARMSERVO);
            }

            if (gamepad1.a) {
                robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad1.b) {
                // continous: -2600
                // cascading: -850
                linearSlideTargetHeight = (int) SLIDE;
                SLIDE_HEIGHT = (int) SLIDE;
                robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
                robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
                robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftSlide.setPower(1);
                robot.rightSlide.setPower(1);
                waitForLinearSlide(linearSlideTargetHeight);
                robot.smartServo.setPosition(0.63);
                robot.arm.setPosition(0.4);
            }
            if (gamepad1.x) {
                linearSlideTargetHeight = 0;
                SLIDE_HEIGHT = 0;
                robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
                robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
                robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftSlide.setPower(0.5);
                robot.rightSlide.setPower(0.5);
                waitForLinearSlide(linearSlideTargetHeight);
            }
        }
//        robot.topLeft.setPosition(robot.topLeft.getPosition());
//        robot.topRight.setPosition(robot.topRight.getPosition());
//        robot.bottomLeft.setPosition(robot.bottomLeft.getPosition());
//        robot.bottomRight.setPosition(robot.bottomRight.getPosition());

    }
    private void waitForLinearSlide(int linearSlideTarget) {
        new Thread(() -> {
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            while ((robot.rightSlide.isBusy() &&
                    robot.rightSlide.isBusy() &&
                    opModeIsActive()) ||
                    runtime.seconds() < 1.5) {
                telemetry.addData("linearSlideTarget", linearSlideTarget);
                telemetry.addData("target", robot.rightSlide.getTargetPosition());
                telemetry.addData("left slide", robot.rightSlide.getCurrentPosition());
                telemetry.addData("right slide", robot.rightSlide.getCurrentPosition());
                telemetry.update();
                idle();
            }

            if (robot.leftSlide.getTargetPosition() == 0) {
                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
            }
        }).start();
    }
}