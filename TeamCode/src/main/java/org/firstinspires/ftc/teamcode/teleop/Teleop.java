package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.max;
import static java.lang.Math.min;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private ElapsedTime runtime = new ElapsedTime();
    public static int SLIDE_HEIGHT = 0;
    TurtleRobot robot = new TurtleRobot(this);
    boolean softlock = true;
    public static double OUTTAKEOPEN =  0; // open
    public static double OUTTAKECLOSE = 1; // close
    public static double SMARTSERVO1 = 0.6;
    public static double SMARTSERVO2 = 0.15;
    public static double TX_PICKUP_SMARTSERVO = 0.46;  //0.4
    public static double HORIZONTALSLIDE = 0;
    public static double BASKET_SMARTSERVO = 1;
    public static double BASKET_ARMSERVO = 0.48;
    public static double TX_PICKUP_ARMSERVO =  0.02; //0.11
    public static double SPEC_DROP_SMART = 0.8;
    public static double SPEC_DROP_ARM = 0;
    public static double OPENINTAKE = 0.7; //0.7
    public static double CLOSEINTAKE = 0.01; //0.01
    public static double TOP_OBSERVE = 0.42; // 0.53
    public static double BOTTOM_OBSERVE = 0.85; //0.85
    public static double TOP_TRANSFER = 0.4;  //0.4
    public static double BOTTOM_TRANSFER = 0.1;  //0.1
    public static double TOP_PICK = 0.48;  // 0.61
    public static double BOTTOM_PICK = 0.85; // 0.8
    public static double BOTTOMINIT = 0.2;
    public static double PICKING_UP = 0.85;
    public static double PADLEFT = 0.1;
    public static double TOP_SCAN_SUB = 0.53;
    public static double BOTTOM_SCAN_SUB = 0.56;
    public static double TOPINIT = TOP_TRANSFER;
    public static double HANG;
    public static double maxmove = 0.8; //0.6
    boolean servolock = false;
    public static double SPEC_PICK_SMARTSERVO = 0.4;
    public static double SPEC_PICK_ARMSERVO = 0.95;
    public static double SLIDE = -1250;
    double BOTTOM_LEFT = BOTTOMINIT;
    double BOTTOM_RIGHT = BOTTOMINIT;
    public static double OFSETRIGHT = 0.01;
    public static double OFSETLEFT = -0.02;



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
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime tim = new ElapsedTime();
        ElapsedTime padb = new ElapsedTime();
        PIDController controller = new PIDController(PIDF.p, PIDF.i, PIDF.d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.topLeft.setPosition(TOPINIT);
        //robot.topRight.setPosition(TOPINIT);
        robot.bottomRight.setPosition(BOTTOMINIT);
        robot.bottomLeft.setPosition(BOTTOMINIT);
        //robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PICKING_UP = 0.85;
        HORIZONTALSLIDE = 0;
        HANG = 0;
        robot.smartServo.setPosition(0.3);
        robot.arm.setPosition(1);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);

        robot.init(hardwareMap);
        robot.bottomLeft.setPosition(0.1);
        robot.bottomRight.setPosition(0.1);

        limelight.pipelineSwitch(0);

        limelight.start();

        waitForStart();

        int linearSlideTargetHeight = 0;

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double divisor = 0.9;
            if (gamepad1.left_trigger != 0) {
                divisor = 2.5;
            }

            robot.leftFront.setPower(Math.cbrt(y + x + rx) / divisor);
            robot.leftBack.setPower(Math.cbrt(y - x + rx) / divisor);
            robot.rightFront.setPower(Math.cbrt(y - x - rx) / divisor);
            robot.rightBack.setPower(Math.cbrt(y + x - rx) / divisor);

            HORIZONTALSLIDE -= gamepad2.left_stick_y / 40;
            HORIZONTALSLIDE = min(HORIZONTALSLIDE, maxmove);
            HORIZONTALSLIDE = max(HORIZONTALSLIDE, 0);

            robot.leftHorizontalSlide.setPosition(HORIZONTALSLIDE);
            robot.rightHorizontalSlide.setPosition(HORIZONTALSLIDE);
            telemetry.addData("horizontal", HORIZONTALSLIDE);

            double actuatorPos = gamepad2.right_trigger - gamepad2.left_trigger;
            robot.rightActuator.setPower(actuatorPos);
            robot.leftActuator.setPower(actuatorPos);

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
            telemetry.addData("intakeclawposition", robot.intake.getPosition());
            if (gamepad2.y) { // Observes
                Log.d("OBSERVE", String.valueOf(robot.topLeft.getPosition()));
                robot.topLeft.setPosition(TOP_OBSERVE);
                robot.intake.setPosition(OPENINTAKE);
//                robot.bottomRight.setPosition(BOTTOM_OBSERVE);
//                robot.bottomLeft.setPosition(BOTTOM_OBSERVE);
                if(gotoobserve) {
                    robot.bottomRight.setPosition(BOTTOM_OBSERVE);
                    robot.bottomLeft.setPosition(BOTTOM_OBSERVE);
                    BOTTOM_LEFT = BOTTOM_OBSERVE;
                    BOTTOM_RIGHT = BOTTOM_OBSERVE;
                    gotoobserve = false;
                }
                double multiplier = 0.015;

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

                if (gamepad2.dpad_right && gamepad2.y) {
                    limelight.pipelineSwitch(1); // blue
                    robot.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }
                if (gamepad2.dpad_up&& gamepad2.y) {
                    limelight.pipelineSwitch(0);
                    robot.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW); // yellow
                }
                if (gamepad2.dpad_down && gamepad2.y) {
                    limelight.pipelineSwitch(2); // red
                    robot.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                }

                if (gamepad2.y && gamepad2.dpad_left) {
                    LLResult result = limelight.getLatestResult();
                    if (result != null) {
                        // Access general information
                        Pose3D botpose = result.getBotpose();
                        double captureLatency = result.getCaptureLatency();
                        double targetingLatency = result.getTargetingLatency();
                        double parseLatency = result.getParseLatency();
                        telemetry.addData("LL Latency", captureLatency + targetingLatency);
                        telemetry.addData("Parse Latency", parseLatency);
                        telemetry.addData("PythonOutput", Arrays.toString(result.getPythonOutput()));

                        if (result.isValid()) {
                            telemetry.addData("tx", result.getTx());
                            telemetry.addData("txnc", result.getTxNC());
                            telemetry.addData("ty", result.getTy());
                            telemetry.addData("tync", result.getTyNC());

                            telemetry.addData("Botpose", botpose.toString());

                            // Access color results
                            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                            for (LLResultTypes.ColorResult cr : colorResults) {
                                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                                telemetry.addData("Area", cr.getTargetArea());

                                /** CLAW ANGLE **/
                                telemetry.addData("corners", cr.getTargetCorners());
                                if (cr.getTargetCorners().size() == 4) {
                                    List<List<Double>> corners = cr.getTargetCorners();
                                    List<List<Double>> ogcorners = cr.getTargetCorners();
                                    Collections.sort(corners, Comparator.comparingDouble(point -> point.get(1)));

                                    List<Double> point1 = corners.get(0); // smallest y
                                    List<Double> point2 = corners.get(1); // second smallest y

                                    double theta;
                                    if (point1.get(0) > point2.get(0)) {
                                        theta = 1;
                                    } else {
                                        theta = -1;
                                    }

                                    // Calculate the angle in radians
                                    theta *= Math.atan(Math.abs(point1.get(1) - point2.get(1)) / Math.abs(point1.get(0) - point2.get(0)));

                                    // Using distance formula
                                    double dist1 = Math.hypot(point1.get(0) - point2.get(0), point1.get(1) - point2.get(1));
                                    int adjIdx = ogcorners.indexOf(point2);
                                    ++adjIdx;
                                    adjIdx %= 3;
                                    List<Double> adjacentPoint = ogcorners.get(adjIdx);
                                    double dist2 = Math.hypot(point2.get(0) - adjacentPoint.get(0), point2.get(1) - adjacentPoint.get(1));

                                    telemetry.addData("dist1", dist1);
                                    telemetry.addData("dist2", dist2);

                                    if (Math.abs(dist1 - dist2) >= 50) {
                                        theta += Math.PI / 2; // adding 90 degrees in radians
                                    }

                                    if (theta < 0) {
                                        theta += Math.PI;
                                    }

                                    if (theta >= Math.PI / 2) {
                                        theta -= Math.PI;
                                    }

                                    theta = min(theta, 90);
                                    theta = max(theta, -90);

                                    // Converting angle to degrees for better understanding
                                    double angleInDegrees = Math.toDegrees(theta);
                                    telemetry.addData("Angle of the sample", angleInDegrees);

                                    robot.bottomRight.setPosition(BOTTOM_OBSERVE + theta / 90 * 5);
                                    robot.bottomLeft.setPosition(BOTTOM_OBSERVE - theta / 90 * 5);
                                    robot.light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                                }
                                telemetry.update();

                            }
                        }
                    }
                }
            }
            if (gamepad2.x) {
                //robot.topRight.setPosition(TOP_TRANSFER);
                robot.topLeft.setPosition(TOP_TRANSFER);
                robot.bottomRight.setPosition(BOTTOM_TRANSFER + OFSETRIGHT);
                robot.bottomLeft.setPosition(BOTTOM_TRANSFER + OFSETLEFT);
                telemetry.addData("Bottom right", robot.bottomRight.getPosition());
                telemetry.addData("Bottom left", robot.bottomLeft.getPosition());
                gotoobserve = true;
            } else if (gamepad2.a) { //scans the submersible
                //robot.topRight.setPosition(TOP_SCAN_SUB);
                robot.topLeft.setPosition(TOP_SCAN_SUB);
                robot.bottomRight.setPosition(BOTTOM_SCAN_SUB);
                robot.bottomLeft.setPosition(BOTTOM_SCAN_SUB);
                robot.intake.setPosition(CLOSEINTAKE);
                gotoobserve = true;
            }

            if (gamepad2.left_bumper) {
                robot.intake.setPosition(OPENINTAKE);
            } else if (gamepad2.right_bumper) {
                robot.intake.setPosition(CLOSEINTAKE);
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
                robot.arm.setPosition(SPEC_PICK_ARMSERVO);
                robot.outtake.setPosition(OUTTAKEOPEN);
                SLIDE_HEIGHT = 0;
            }

            if (gamepad1.dpad_down) { // outtake action - pick up from transfer box
                robot.smartServo.setPosition(BASKET_SMARTSERVO);
                robot.arm.setPosition(BASKET_ARMSERVO);
                SLIDE_HEIGHT = -750;
                robot.outtake.setPosition(OUTTAKEOPEN);
            }

            if (gamepad1.dpad_up) { // outtake action - drop position for basket
                robot.smartServo.setPosition(BASKET_SMARTSERVO);
                robot.arm.setPosition(BASKET_ARMSERVO);
            }

            if (gamepad1.dpad_left) { // outtake action - pick specimen drop
                robot.outtake.setPosition(OUTTAKECLOSE);
                robot.smartServo.setPosition(0);
                robot.arm.setPosition(SPEC_DROP_ARM);
                SLIDE_HEIGHT = -640;
                tim.reset();

            }
            if (tim.milliseconds() >= 200 && tim.milliseconds() <= 300) {
                robot.smartServo.setPosition(SPEC_DROP_SMART);
            }
            if (gamepad1.a) {
                robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
                robot.arm.setPosition(TX_PICKUP_ARMSERVO+0.1);
                robot.outtake.setPosition(OUTTAKEOPEN);
                SLIDE_HEIGHT = 0;
            }
            if (gamepad1.y) {
                SLIDE_HEIGHT = -1350;
                if (robot.leftSlide.getCurrentPosition() <= -1250) {
                    robot.outtake.setPosition(OUTTAKEOPEN);
                }
            }
            if (gamepad1.b) {
                robot.outtake.setPosition(OUTTAKECLOSE);
                padb.reset();
                // continuous: -2600
                // cascading: -850
            }

            if (padb.milliseconds() >= 50 && padb.milliseconds() <= 100) {
                SLIDE_HEIGHT = -2100;
                robot.smartServo.setPosition(BASKET_SMARTSERVO);
                robot.arm.setPosition(BASKET_ARMSERVO);
            }

            if (gamepad1.x) {
                robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
                robot.arm.setPosition(TX_PICKUP_ARMSERVO);
                robot.outtake.setPosition(OUTTAKEOPEN);
                SLIDE_HEIGHT = 0;
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
//        robot.topLeft.setPosition(robot.topLeft.getPosition());
//        robot.topRight.setPosition(robot.topRight.getPosition());

    }
}
