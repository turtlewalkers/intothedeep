package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.max;
import static java.lang.Math.min;

import android.graphics.Color;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Config
@TeleOp
public class Teleop extends LinearOpMode {
    private static boolean CHECK = false;
    private ElapsedTime runtime = new ElapsedTime();
    public static int SLIDE_HEIGHT = 0;
    TurtleRobot robot = new TurtleRobot(this);
    boolean softlock = true;
    public static double OUTTAKEOPEN = 1; // open
    public static double OUTTAKECLOSE = 0; // close
    public static double SMARTSERVO1 = 0.6;
    public static double SMARTSERVO2 = 0.15;
    public static double OFFSET_SMARTSERVO = 0.3;
    public static double TX_PICKUP_SMARTSERVO = 1; //0.4
    public static double HORIZONTALSLIDE = 0;
    public static double BASKET_SMARTSERVO = 0;
    public static double BASKET_ARMSERVO = 0.43;
    public static double TX_PICKUP_ARMSERVO = 0.12; //0.11
    public static double SPEC_DROP_SMART = 0.46;
    public static double SPEC_DROP_ARM = 0.13;
    public static double SMART_SERVO_FLEX = 0.65;
    public static double OPENINTAKE = 0.11; //0.7
    public static double CLOSEINTAKE = 0.29; //0.01
    public static double TOP_OBSERVE = 0.22; // 0.53
    public static double BOTTOM_OBSERVE = 0.46; //0.85
    public static double TOP_TRANSFER = 0;  //0.4
    public static double BOTTOM_TRANSFER = 0.13;  //0.1
    public static double TOP_PICK = 0.345;  // 0.61
    public static double BOTTOM_PICK = 0.85; // 0.8
    public static double BOTTOMINIT = 0.2;
    public static double PICKING_UP = 0.85;
    public static double PADLEFT = 0.1;
    public static double TOP_SCAN_SUB = 0.22;
    public static double BOTTOM_SCAN_SUB = 0.22;
    public static double TOPINIT = TOP_TRANSFER;
    public static double HANG = 0;
    public static double maxmove = 0.8; //0.6
    boolean servolock = false;
    public static double SPEC_PICK_SMARTSERVO = 0.6;
    public static double SPEC_PICK_ARMSERVO = 0.94;
    public static double TELE_PICK_ARM = 0.9;
    public static double SLIDE = -1250;
    double BOTTOM_LEFT = BOTTOMINIT;
    double BOTTOM_RIGHT = BOTTOMINIT;
    public static double OFSETRIGHT = 0.01;
    public static double OFSETLEFT = -0.02;
    public static double SPEC_SERVO_PICK = 0.78;
    public static double SPEC_SERVO_DROP = 0.04;
    public static double SPEC_TRANSFER = 0.78; //heeheehee
    public static double x1 = 0;
    public static int x2 = -100;
    public static int x3 = -620; //-380
    public static double SPACE_LIMIT;
    double actuatorPos = 0;


    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;

    static final double PULLEY_DIAMETER_INCHES = 1.404;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (PULLEY_DIAMETER_INCHES * Math.PI);

    private static final int FOCAL_LENGTH = 1320;
    private static final double WIDTH = 3.5;
    double left_command;
    double right_command;
    boolean gotoobserve = true;
//    private Limelight3A limelight;

    private int red;
    private int blue;
    private int green;
    private double volts;
    private double hangStrength = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime tim = new ElapsedTime();
        ElapsedTime padb = new ElapsedTime();
        ElapsedTime drop = new ElapsedTime();
        ElapsedTime getvoltage = new ElapsedTime();
        ElapsedTime trime = new ElapsedTime();
        ElapsedTime timerGamePadA = new ElapsedTime();
        ElapsedTime hang = new ElapsedTime();
        PIDController controller = new PIDController(PIDF.p, PIDF.i, PIDF.d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        robot.topLeft.setPosition(TOPINIT);
        //robot.topRight.setPosition(TOPINIT);
        robot.bottomRight.setPosition(BOTTOMINIT);
        robot.bottomLeft.setPosition(BOTTOMINIT);
        //robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PICKING_UP = 0.85;
        HORIZONTALSLIDE = 0;
        robot.smartServo.setPosition(1);
        robot.arm.setPosition(SPEC_PICK_ARMSERVO);

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);

        //robot.init(hardwareMap);
        robot.bottomLeft.setPosition(0.1);
        robot.bottomRight.setPosition(0.1);
        ColorSensor color = hardwareMap.get(ColorSensor.class, "Color");

        robot.intake.setPosition(CLOSEINTAKE);
        int startcolor = 0;
        double positiona = robot.analogInput.getVoltage();
        CHECK = false;
        hangStrength = 1;
//        limelight.pipelineSwitch(0);
//        limelight.start();

        waitForStart();

        int linearSlideTargetHeight = 0;

        while (opModeIsActive()) {
//            LLStatus status = limelight.getStatus();
//            telemetry.addData("Name", "%s",
//                    status.getName());
//            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                    status.getTemp(), status.getCpu(), (int) status.getFps());
//            telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                    status.getPipelineIndex(), status.getPipelineType());
            if (startcolor == 0) {
                startcolor += 1;
            }
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double divisor = 1;
            if (gamepad1.left_trigger != 0) {
                divisor = 2.5;
            }

            robot.leftFront.setPower(Math.cbrt(y + x + rx) / divisor);
            robot.leftBack.setPower(Math.cbrt(y - x + rx) / divisor);
            robot.rightFront.setPower(Math.cbrt(y - x - rx) / divisor);
            robot.rightBack.setPower(Math.cbrt(y + x - rx) / divisor);

            if (!CHECK) {
                HORIZONTALSLIDE -= gamepad2.left_stick_y / 20 + gamepad2.right_stick_y / 80;
                HORIZONTALSLIDE = min(HORIZONTALSLIDE, maxmove);
                HORIZONTALSLIDE = max(HORIZONTALSLIDE, 0);

                robot.leftHorizontalSlide.setPosition(HORIZONTALSLIDE);
                robot.rightHorizontalSlide.setPosition(HORIZONTALSLIDE);
                telemetry.addData("horizontal", HORIZONTALSLIDE);
            }

            if (!CHECK) {
                actuatorPos = gamepad2.right_trigger - gamepad2.left_trigger;
            }

            if (gamepad2.dpad_up) {
                CHECK = true;
            }

            if (CHECK) {
                if (CHECK && hang.milliseconds() >= 5000) {
                    hangStrength = 0.6;
                } else {
                    hangStrength = 1;
                }
                if (CHECK && hang.milliseconds() >= 10000) {
                    hangStrength = 0;
                }
                actuatorPos = hangStrength;
                hang.reset();
                robot.rightHorizontalSlide.setPosition(0.2);
                robot.leftHorizontalSlide.setPosition(0.2);
            }
            robot.rightActuator.setPower(actuatorPos);
            robot.leftActuator.setPower(actuatorPos);

            if (getvoltage.milliseconds() >= 100) {
                volts = robot.analogInput.getVoltage();
                getvoltage.reset();

                red = color.red();
                blue = color.blue();
                green = color.green();
                if (volts <= 2.54) {
                    robot.light.setPosition(1);
                } else if (red > blue && red > green && red > 100) {
                    robot.light.setPosition(0.279);
                } else if (blue > red && blue > green && blue > 100) {
                    robot.light.setPosition(0.631);
                } else if (green > blue && red < green && green > 120) {
                    robot.light.setPosition(0.388);
                } else if (volts >= 2.66 && volts <= 2.77) {
                    robot.light.setPosition(0.524);
                } else {
                    robot.light.setPosition(0);
                }
            }

            telemetry.addData("voltage ", volts);
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);

            if (gamepad2.b) { // Picks
                robot.topLeft.setPosition(TOP_PICK);
                robot.bottomRight.setPosition(robot.bottomRight.getPosition());
                robot.bottomLeft.setPosition(robot.bottomLeft.getPosition());

                timer.reset();

            }
//

            if (timer.milliseconds() >= 50 && timer.milliseconds() <= 100) {
                robot.intake.setPosition(CLOSEINTAKE);
            }

            if (gamepad2.y) { // Observes
                robot.topLeft.setPosition(TOP_OBSERVE);
                robot.intake.setPosition(OPENINTAKE);
                if (gotoobserve) {
                    robot.bottomRight.setPosition(BOTTOM_OBSERVE);
                    robot.bottomLeft.setPosition(BOTTOM_OBSERVE);
                    BOTTOM_LEFT = BOTTOM_OBSERVE;
                    BOTTOM_RIGHT = BOTTOM_OBSERVE;
                    gotoobserve = false;
                }
                double multiplier = 0.03;

                if (!gamepad2.dpad_left) {
                    if (gamepad2.right_stick_x != 0) {
                        BOTTOM_LEFT += gamepad2.right_stick_x * multiplier;
                        BOTTOM_RIGHT -= gamepad2.right_stick_x * multiplier;
                    }

                    BOTTOM_LEFT = min(BOTTOM_LEFT, BOTTOM_OBSERVE + 0.24);
                    BOTTOM_LEFT = max(BOTTOM_LEFT, BOTTOM_OBSERVE - 0.24);
                    BOTTOM_RIGHT = max(BOTTOM_RIGHT, BOTTOM_OBSERVE - 0.24);
                    BOTTOM_RIGHT = min(BOTTOM_RIGHT, BOTTOM_OBSERVE + 0.24);

                    robot.bottomRight.setPosition(BOTTOM_RIGHT);
                    robot.bottomLeft.setPosition(BOTTOM_LEFT);

                    telemetry.addData("Bottom right", BOTTOM_LEFT);
                    telemetry.addData("Bottom left", BOTTOM_RIGHT);
                }

//                if (gamepad2.dpad_right && gamepad2.y) {
//                    limelight.pipelineSwitch(1); // blue
//                }
//                if (gamepad2.dpad_up&& gamepad2.y) { //yellow
//                    limelight.pipelineSwitch(0);
//                }
//                if (gamepad2.dpad_down && gamepad2.y) {
//                    limelight.pipelineSwitch(2); // red
//                }
//
//                if (gamepad2.y && gamepad2.dpad_left) {
//                    LLResult result = limelight.getLatestResult();
//                    if (result != null) {
//                        // Access general information
//                        Pose3D botpose = result.getBotpose();
//                        double captureLatency = result.getCaptureLatency();
//                        double targetingLatency = result.getTargetingLatency();
//                        double parseLatency = result.getParseLatency();
//                        telemetry.addData("LL Latency", captureLatency + targetingLatency);
//                        telemetry.addData("Parse Latency", parseLatency);
//                        telemetry.addData("PythonOutput", Arrays.toString(result.getPythonOutput()));
//
//                        if (result.isValid()) {
//                            telemetry.addData("tx", result.getTx());
//                            telemetry.addData("txnc", result.getTxNC());
//                            telemetry.addData("ty", result.getTy());
//                            telemetry.addData("tync", result.getTyNC());
//
//                            telemetry.addData("Botpose", botpose.toString());
//
//                            // Access color results
//                            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//                            for (LLResultTypes.ColorResult cr : colorResults) {
//                                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
//                                telemetry.addData("Area", cr.getTargetArea());
//
//                                /** CLAW ANGLE **/
//                                telemetry.addData("corners", cr.getTargetCorners());
//                                if (cr.getTargetCorners().size() == 4) {
//                                    List<List<Double>> corners = cr.getTargetCorners();
//                                    List<List<Double>> ogcorners = cr.getTargetCorners();
//                                    Collections.sort(corners, Comparator.comparingDouble(point -> point.get(1)));
//
//                                    List<Double> point1 = corners.get(0); // smallest y
//                                    List<Double> point2 = corners.get(1); // second smallest y
//
//                                    double theta;
//                                    if (point1.get(0) > point2.get(0)) {
//                                        theta = 1;
//                                    } else {
//                                        theta = -1;
//                                    }
//
//                                    // Calculate the angle in radians
//                                    theta *= Math.atan(Math.abs(point1.get(1) - point2.get(1)) / Math.abs(point1.get(0) - point2.get(0)));
//
//                                    // Using distance formula
//                                    double dist1 = Math.hypot(point1.get(0) - point2.get(0), point1.get(1) - point2.get(1));
//                                    int adjIdx = ogcorners.indexOf(point2);
//                                    ++adjIdx;
//                                    adjIdx %= 3;
//                                    List<Double> adjacentPoint = ogcorners.get(adjIdx);
//                                    double dist2 = Math.hypot(point2.get(0) - adjacentPoint.get(0), point2.get(1) - adjacentPoint.get(1));
//
//                                    telemetry.addData("dist1", dist1);
//                                    telemetry.addData("dist2", dist2);
//
//                                    if (Math.abs(dist1 - dist2) >= 50) {
//                                        theta += Math.PI / 2; // adding 90 degrees in radians
//                                    }
//
//                                    if (theta < 0) {
//                                        theta += Math.PI;
//                                    }
//
//                                    if (theta >= Math.PI / 2) {
//                                        theta -= Math.PI;
//                                    }
//
//                                    theta = min(theta, 90);
//                                    theta = max(theta, -90);
//
//                                    // Converting angle to degrees for better understanding
//                                    double angleInDegrees = Math.toDegrees(theta);
//                                    telemetry.addData("Angle of the sample", angleInDegrees);
//
//                                    robot.bottomRight.setPosition(BOTTOM_OBSERVE + theta / 90 * 5);
//                                    robot.bottomLeft.setPosition(BOTTOM_OBSERVE - theta / 90 * 5);
////                                    robot.light.setPosition(0.500);
//                                }
//                                telemetry.update();
//
//                            }
            }
            if (gamepad2.x) {
                robot.topLeft.setPosition(TOP_TRANSFER);
                robot.bottomRight.setPosition(BOTTOM_TRANSFER + OFSETRIGHT);
                robot.bottomLeft.setPosition(BOTTOM_TRANSFER + OFSETLEFT);
                telemetry.addData("Bottom right", robot.bottomRight.getPosition());
                telemetry.addData("Bottom left", robot.bottomLeft.getPosition());
                gotoobserve = true;
            }

            if (gamepad2.a) { //scans the submersible
                robot.topLeft.setPosition(TOP_SCAN_SUB);
                robot.bottomRight.setPosition(BOTTOM_SCAN_SUB);
                robot.bottomLeft.setPosition(BOTTOM_SCAN_SUB);
                robot.intake.setPosition(CLOSEINTAKE);
                gotoobserve = true;
            }

            if (gamepad2.left_bumper) {
                robot.intake.setPosition(CLOSEINTAKE);
            } else if (gamepad2.right_bumper) {
                robot.intake.setPosition(OPENINTAKE);
            }

//            if (gamepad2.dpad_left) {
//                BOTTOM_LEFT = PADLEFT;
//                BOTTOM_RIGHT = PADLEFT;
//            } else if (gamepad2.dpad_down && softlock) {
//                BOTTOM_LEFT = 0.8;
//                BOTTOM_RIGHT = 0.8;
//            }

            if (gamepad1.right_bumper) {
                robot.outtake.setPosition(OUTTAKECLOSE);
            } else if (gamepad1.left_bumper) {
                robot.outtake.setPosition(OUTTAKEOPEN);
            }

            if (gamepad1.dpad_right) { // outtake action - pick specimen from wall
                robot.smartServo.setPosition(SPEC_PICK_SMARTSERVO);
                robot.arm.setPosition(TELE_PICK_ARM);
                robot.spec.setPosition(SPEC_SERVO_PICK);
                robot.outtake.setPosition(OUTTAKEOPEN);
                SLIDE_HEIGHT = 0;
            }

            if (gamepad1.dpad_down) { // outtake action - pick up from transfer box
                robot.smartServo.setPosition(BASKET_SMARTSERVO);
                robot.arm.setPosition(BASKET_ARMSERVO);
                SLIDE_HEIGHT = -750;
                robot.spec.setPosition(SPEC_TRANSFER);
                robot.outtake.setPosition(OUTTAKEOPEN);
            }

            if (gamepad1.dpad_up) { // outtake action - drop position for basket
                robot.smartServo.setPosition(BASKET_SMARTSERVO);
                robot.arm.setPosition(BASKET_ARMSERVO);
            }

            if (gamepad1.dpad_left) { // outtake action - pick specimen drop
                robot.outtake.setPosition(OUTTAKECLOSE);
                tim.reset();

            }
            if (tim.milliseconds() >= 90 && tim.milliseconds() <= 100) {
                robot.smartServo.setPosition(1);
            }
            if (tim.milliseconds() >= 100 && tim.milliseconds() <= 150) {
                SLIDE_HEIGHT = x3;
                robot.spec.setPosition(SPEC_SERVO_DROP);
                robot.arm.setPosition(SPEC_DROP_ARM);
            }
            if (tim.milliseconds() >= 350 && tim.milliseconds() <= 400) {
                robot.smartServo.setPosition(SPEC_DROP_SMART);
            }
            if (gamepad1.a) {
                robot.outtake.setPosition(OUTTAKEOPEN);
                timerGamePadA.reset();
            }
            if (timerGamePadA.milliseconds() >= 150 && timerGamePadA.milliseconds() < 200) {
                robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
                robot.arm.setPosition(TX_PICKUP_ARMSERVO + 0.1);
                robot.spec.setPosition(SPEC_TRANSFER);
                SLIDE_HEIGHT = 0;
            }
            if (gamepad1.y) {
//                SLIDE_HEIGHT = -1000;
                //robot.arm.setPosition(SPEC_DROP_ARM + x1);
                //robot.smartServo.setPosition(SPEC_PICK_SMARTSERVO - x4);
                SLIDE_HEIGHT = x2;
                robot.arm.setPosition(SPEC_DROP_ARM-0.1);
                drop.reset();
            }

            /*
            if (drop.milliseconds() >= 75 && drop.milliseconds() <= 125) {
                robot.outtake.setPosition(OUTTAKEOPEN);
            }*/

            if (drop.milliseconds() >= 5 && drop.milliseconds() <= 20) {
                robot.outtake.setPosition(OUTTAKEOPEN);
            }

            if (gamepad1.b) {
                robot.outtake.setPosition(OUTTAKECLOSE);
                padb.reset();
                // continuous: -2600
                // cascading: -850
            }

            if (gamepad2.start) {
                robot.topLeft.setPosition(0);
                robot.bottomLeft.setPosition(BOTTOM_TRANSFER-0.05);
            }

            if (padb.milliseconds() >= 50 && padb.milliseconds() <= 100) {
                SLIDE_HEIGHT = -2100;
                robot.smartServo.setPosition(BASKET_SMARTSERVO);
                robot.arm.setPosition(BASKET_ARMSERVO);
            }

            if (gamepad1.x) {
                robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
                robot.arm.setPosition(TX_PICKUP_ARMSERVO+0.1);
                robot.outtake.setPosition(OUTTAKEOPEN);
                robot.spec.setPosition(SPEC_TRANSFER);
                SLIDE_HEIGHT = 0;
                trime.reset();
            }
            if (trime.milliseconds() >= 50 && trime.milliseconds() <= 80) {
                robot.intake.setPosition(OPENINTAKE);
            }
            if (trime.milliseconds() >= 81 && trime.milliseconds() <= 150 ) {
                robot.arm.setPosition(TX_PICKUP_ARMSERVO);
            }
            if (trime.milliseconds() >= 151 && trime.milliseconds() <= 281 ) {
                robot.outtake.setPosition(OUTTAKECLOSE);
            }


            controller.setPID(PIDF.p, PIDF.i, PIDF.d);
            int linearSlidePosition = robot.leftSlide.getCurrentPosition();
            double pid = controller.calculate(linearSlidePosition, SLIDE_HEIGHT);
            double ff = Math.toRadians(SLIDE_HEIGHT / PIDF.ticks_in_degrees) * PIDF.f;
            double power = pid + ff;

            robot.leftSlide.setPower(power);
            robot.rightSlide.setPower(power);

            telemetry.addData("pos", linearSlidePosition);
            telemetry.addData("target", SLIDE_HEIGHT);
            telemetry.addData("power", power);
            telemetry.addData("bottom left", robot.bottomLeft.getPosition());
            telemetry.addData("bottom right", robot.bottomRight.getPosition());
            telemetry.update();

        }
    }
}