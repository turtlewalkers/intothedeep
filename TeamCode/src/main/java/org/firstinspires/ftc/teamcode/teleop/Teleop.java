package org.firstinspires.ftc.teamcode.teleop;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;
import org.firstinspires.ftc.teamcode.teleop.PIDF;

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
    public static double CONSTANT;
    public static double OUTTAKECLAW1 = 1;
    public static double OUTTAKECLAW2 = 0;
    public static double SMARTSERVO1 = 0.6;
    public static double SMARTSERVO2 = 0.15;
    public static double TX_PICKUP_SMARTSERVO = 0.7;
    public static double BASKET_SMARTSERVO = 0.88;
    public static double BASKET_ARMSERVO = 0.52;
    public static double TX_PICKUP_ARMSERVO = 0;
    public static double TOPINIT = 0;
    public static double OPENINTAKE = 1;
    public static double CLOSEINTAKE = 0;
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
    public static double MAX_POWER_SLIDE = 0.5;
    public static boolean GOING_UP = false;
    boolean servolock = false;
    public static double SPEC_PICK_SMARTSERVO = 0.41;
    public static double SPEC_PICK_ARMSERVO = 0.17;
    public static double SLIDE = -1250;
    double BOTTOM_LEFT = BOTTOMINIT;
    double BOTTOM_RIGHT = BOTTOMINIT;
    double HORIZONTALSLIDE;

    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double PULLEY_DIAMETER_INCHES = 1.404;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (PULLEY_DIAMETER_INCHES * Math.PI);

    private static final int FOCAL_LENGTH = 1320;
    private static final double WIDTH = 3.5;
    double left_command;
    double right_command;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        PIDController controller = new PIDController(PIDF.p, PIDF.i, PIDF.d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

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
//                robot.leftSlide.setPower(gamepad2.right_stick_y);
//                robot.rightSlide.setPower(gamepad2.right_stick_y);
            }
            if (gamepad2.b && servolock == false) { //Picks
                robot.topRight.setPosition(TOP_PICK);
                robot.topLeft.setPosition(TOP_PICK);
                sleep(200);
                robot.intake.setPosition(CLOSEINTAKE);
            } else if (gamepad2.y && servolock == false) { // Observes
                robot.topRight.setPosition(TOP_OBSERVE);
                robot.topLeft.setPosition(TOP_OBSERVE);
                robot.intake.setPosition(OPENINTAKE);
                robot.bottomRight.setPosition(BOTTOM_OBSERVE);
                robot.bottomLeft.setPosition(BOTTOM_OBSERVE);
                double multiplier = 0.1;

                if (gamepad2.dpad_right && gamepad2.y) {
                    limelight.pipelineSwitch(1);
                } if (gamepad2.dpad_up && gamepad2.y) {
                    limelight.pipelineSwitch(0);
                } if (gamepad2.dpad_down && gamepad2.y) {
                    limelight.pipelineSwitch(2);
                }

                if (gamepad2.dpad_left && gamepad2.y) {
                    LLResult result = limelight.getLatestResult();
                    if (result != null) {
                        // Access general information
                        Pose3D botpose = result.getBotpose();
                        double captureLatency = result.getCaptureLatency();
                        double targetingLatency = result.getTargetingLatency();
                        double parseLatency = result.getParseLatency();
                        telemetry.addData("LL Latency", captureLatency + targetingLatency);
                        telemetry.addData("Parse Latency", parseLatency);
                        telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

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

                                    // Converting angle to degrees for better understanding
                                    double angleInDegrees = Math.toDegrees(theta);
                                    telemetry.addData("Angle of the sample", angleInDegrees);

                                    robot.bottomRight.setPosition(0.8 + theta / Math.PI * 0.1);
                                    robot.bottomLeft.setPosition(0.8 - theta / Math.PI * 0.1);
                                }
                                telemetry.update();

                            }
                        }
                    }
                }

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
                SLIDE_HEIGHT = -625;
                robot.smartServo.setPosition(BASKET_SMARTSERVO);
                robot.arm.setPosition(BASKET_ARMSERVO);
            }
            if (gamepad1.b) {
                SLIDE_HEIGHT = -1250;
                robot.smartServo.setPosition(BASKET_SMARTSERVO);
                robot.arm.setPosition(BASKET_ARMSERVO);
                // continuous: -2600
                // cascading: -850
            }
            if (gamepad1.x) {
                robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
                robot.arm.setPosition(TX_PICKUP_ARMSERVO);
                SLIDE_HEIGHT = 0;
            }

            if (SLIDE_HEIGHT != 0) {
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
                telemetry.update();
            } else {
                robot.leftSlide.setPower(0.12);
                robot.rightSlide.setPower(0.12);
            }
        }
//        robot.topLeft.setPosition(robot.topLeft.getPosition());
//        robot.topRight.setPosition(robot.topRight.getPosition());
//        robot.bottomLeft.setPosition(robot.bottomLeft.getPosition());
//        robot.bottomRight.setPosition(robot.bottomRight.getPosition());

    }
}