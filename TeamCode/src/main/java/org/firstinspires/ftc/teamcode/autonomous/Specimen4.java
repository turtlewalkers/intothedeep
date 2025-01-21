package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.teleop.PIDF.d;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.i;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.p;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOM_OBSERVE;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOM_PICK;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOM_TRANSFER;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.CLOSEINTAKE;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OFSETLEFT;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OFSETRIGHT;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OPENINTAKE;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OUTTAKEOPEN;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OUTTAKECLOSE;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_DROP_ARM;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_DROP_SMART;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_PICK_ARMSERVO;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_PICK_SMARTSERVO;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_SERVO_DROP;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_SERVO_PICK;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOP_OBSERVE;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOP_PICK;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOP_SCAN_SUB;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOP_TRANSFER;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TX_PICKUP_ARMSERVO;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TX_PICKUP_SMARTSERVO;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.x2;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.x3;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Drawing;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Config
@Autonomous
public class Specimen4 extends LinearOpMode {
    private static int SLIDE_HEIGHT;
    TurtleRobot robot = new TurtleRobot(this);
    private ElapsedTime timeElapsed;
    private ElapsedTime STOP;

    private Follower follower;
    private PathChain paths, path8;
    private Path path1, path2, path3, path4, path5, path6, path7, path11, path10;
    private Path path9;
    private PIDController controller;

    private Path sample1, sample2, sample3, sample4;

    public static double sample1x = 21;
    public static double sample1y = 30;
    public static double sample2x = 20;
    public static double sample2y = 17;
    public static double sample3x = 21;
    public static double sample3y = 16;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        robot.init(hardwareMap);
        STOP = new ElapsedTime();
        robot.topLeft.setPosition(0);
        robot.bottomRight.setPosition(BOTTOM_TRANSFER);
        robot.bottomLeft.setPosition(BOTTOM_TRANSFER);
//        robot.smartServo.setPosition(SPEC_DROP_SMART);
//        robot.arm.setPosition(SPEC_DROP_ARM);
        robot.outtake.setPosition(OUTTAKECLOSE);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7, 57, 0));
        timeElapsed = new ElapsedTime();

        path1 = new Path(new BezierCurve(new Point(7, 57, Point.CARTESIAN), new Point(32, 72, Point.CARTESIAN)));
        path1.setConstantHeadingInterpolation(0);

//        path2 = new Path(new BezierCurve(new Point(32, 68, Point.CARTESIAN), new Point(5, 25, Point.CARTESIAN), new Point(55, 36, Point.CARTESIAN)));
//        path2.setConstantHeadingInterpolation(0);

        paths = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(32, 72, Point.CARTESIAN),
                        new Point(14, 25, Point.CARTESIAN),
                        new Point(55, 36, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(55, 36, Point.CARTESIAN),
                        new Point(50, 27, Point.CARTESIAN),
                        new Point(26, 26, Point.CARTESIAN))) // path3
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(26, 26, Point.CARTESIAN),
                        new Point(70, 25, Point.CARTESIAN),
                        new Point(50, 17, Point.CARTESIAN))) // path4
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(50, 17, Point.CARTESIAN),
                        new Point(26, 17, Point.CARTESIAN))) // path5
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(26, 15, Point.CARTESIAN),
                        new Point(70, 14, Point.CARTESIAN),
                        new Point(50, 10, Point.CARTESIAN))) // path6
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(50, 10, Point.CARTESIAN),
                        new Point(30, 10, Point.CARTESIAN))) // path7
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(30, 10, Point.CARTESIAN),
                        new Point(30, 46, Point.CARTESIAN),
                        new Point(9, 39, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)

                .build();

        path3 = new Path(new BezierCurve(new Point(63, 22, Point.CARTESIAN), new Point(20, 22, Point.CARTESIAN)));
        path3.setConstantHeadingInterpolation(0);

        path4 = new Path(new BezierCurve(new Point(20, 22, Point.CARTESIAN), new Point(70, 43, Point.CARTESIAN), new Point(60, 17, Point.CARTESIAN)));
        path4.setConstantHeadingInterpolation(0);

        path5 = new Path(new BezierCurve(new Point(60, 17, Point.CARTESIAN), new Point(20, 17, Point.CARTESIAN)));
        path5.setConstantHeadingInterpolation(0);

        path6 = new Path(new BezierCurve(new Point(20, 15, Point.CARTESIAN), new Point(70, 35, Point.CARTESIAN), new Point(60, 10, Point.CARTESIAN)));
        path6.setConstantHeadingInterpolation(0);

        path7 = new Path(new BezierCurve(new Point(60, 9, Point.CARTESIAN), new Point(20, 9, Point.CARTESIAN)));
        path7.setConstantHeadingInterpolation(0);

//        path8 = new Path(new BezierCurve(new Point(20, 9, Point.CARTESIAN), new Point(60, 15, Point.CARTESIAN), new Point(12, 34, Point.CARTESIAN)));
//        path8.setConstantHeadingInterpolation(0);

        path8 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(20, 9, Point.CARTESIAN), new Point(40, 20, Point.CARTESIAN), new Point(10, 36, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)
                .build();

        path9 = new Path(new BezierCurve(new Point(7, 39, Point.CARTESIAN),
                new Point(42, 45, Point.CARTESIAN),
                new Point(32, 68, Point.CARTESIAN)));
        path9.setConstantHeadingInterpolation(0);

        path10 = new Path(new BezierCurve(new Point(35, 68, Point.CARTESIAN),
                new Point(26,39, Point.CARTESIAN),
                new Point(22, 36, Point.CARTESIAN)));
        path10.setConstantHeadingInterpolation(0);

//        path11 = new Path(new BezierCurve(new Point(17, 38, Point.CARTESIAN), new Point(14, 36, Point.CARTESIAN)));
//        path11.setConstantHeadingInterpolation(0);

//        path10 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(33, 68, Point.CARTESIAN),
////                                new Point(10, 23, Point.CARTESIAN),
////                        new Point(23, 38, Point.CARTESIAN),
//                        new Point(17, 38, Point.CARTESIAN)))
//                .setConstantHeadingInterpolation(0)
//                .build();

        sample1 = new Path(new BezierCurve(new Point(35, 70, Point.CARTESIAN),
                new Point(sample1x, sample1y, Point.CARTESIAN)));
        sample1.setLinearHeadingInterpolation(0, 0);

        sample2 = new Path(new BezierCurve(new Point(sample1x, sample1y, Point.CARTESIAN),
                new Point(sample2x, sample2y, Point.CARTESIAN)));
        sample2.setConstantHeadingInterpolation(0);

        sample3 = new Path(new BezierCurve(new Point(sample2x, sample2y, Point.CARTESIAN),
                new Point(sample3x, sample3y, Point.CARTESIAN)));
        sample3.setConstantHeadingInterpolation(-Math.PI/6);
        sample4 = new Path(new BezierCurve(new Point(sample3x, sample3y, Point.CARTESIAN),
                new Point(19, 33, Point.CARTESIAN),
                new Point(13, 33, Point.CARTESIAN)));
        sample4.setConstantHeadingInterpolation(0);

        waitForStart();
        SLIDE_HEIGHT = x3;
        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSlide.setPower(1);
        robot.rightSlide.setPower(1);
        waitForLinearSlide(SLIDE_HEIGHT);
        robot.outtake.setPosition(OUTTAKECLOSE);
        robot.smartServo.setPosition(SPEC_DROP_SMART);
        robot.arm.setPosition(SPEC_DROP_ARM);
        robot.spec.setPosition(SPEC_SERVO_DROP);
        robot.topLeft.setPosition(TOP_SCAN_SUB);
        robot.bottomRight.setPosition(BOTTOM_TRANSFER + OFSETRIGHT);
        robot.bottomLeft.setPosition(BOTTOM_TRANSFER + OFSETLEFT);
        followPath(path1, false);
        SLIDE_HEIGHT = x2;
        robot.arm.setPosition(SPEC_DROP_ARM-0.07);
        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSlide.setPower(1);
        robot.rightSlide.setPower(1);
        waitForLinearSlide(SLIDE_HEIGHT);
        sleep(200);
        robot.outtake.setPosition(OUTTAKEOPEN);
        SLIDE_HEIGHT = 0;
        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSlide.setPower(1);
        robot.rightSlide.setPower(1);
        waitForLinearSlide(SLIDE_HEIGHT);
        robot.smartServo.setPosition(SPEC_PICK_SMARTSERVO);
        robot.arm.setPosition(SPEC_PICK_ARMSERVO);
        robot.spec.setPosition(SPEC_SERVO_PICK);
//        followPath(path2);

        followPath(paths, true);

//        followPath(path8);
//        follower.setMaxPower(0.9);
        for (int i = 0; i < 4; ++i) {
            robot.outtake.setPosition(OUTTAKECLOSE);
            sleep(100);
            SLIDE_HEIGHT = x3;
            robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftSlide.setPower(1);
            robot.rightSlide.setPower(1);
            waitForLinearSlide(SLIDE_HEIGHT);
            robot.outtake.setPosition(OUTTAKECLOSE);
            robot.smartServo.setPosition(SPEC_DROP_SMART);
            robot.arm.setPosition(SPEC_DROP_ARM);
            robot.spec.setPosition(SPEC_SERVO_DROP);
            robot.topLeft.setPosition(TOP_SCAN_SUB);
            robot.bottomRight.setPosition(BOTTOM_TRANSFER + OFSETRIGHT);
            robot.bottomLeft.setPosition(BOTTOM_TRANSFER + OFSETLEFT);
            followPath(path1, false);
            SLIDE_HEIGHT = x2;
            robot.arm.setPosition(SPEC_DROP_ARM-0.07);
            robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftSlide.setPower(1);
            robot.rightSlide.setPower(1);
            waitForLinearSlide(SLIDE_HEIGHT);
            sleep(200);
            robot.outtake.setPosition(OUTTAKEOPEN);
            SLIDE_HEIGHT = 0;
            robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftSlide.setPower(1);
            robot.rightSlide.setPower(1);
            waitForLinearSlide(SLIDE_HEIGHT);
            robot.smartServo.setPosition(SPEC_PICK_SMARTSERVO);
            robot.arm.setPosition(SPEC_PICK_ARMSERVO);
            robot.spec.setPosition(SPEC_SERVO_PICK);
            followPath(path10, true);
//            follower.setMaxPower(0.9);
        }
    }
    public void followPath(PathChain path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        while (follower.isBusy())
            UpdatePathAndTelemetry();
    }
    public void followPath(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
        while (follower.isBusy())
            UpdatePathAndTelemetry();
    }
    private void UpdatePathAndTelemetry(){
        follower.update();
        Drawing.drawRobot(follower.getPose(), "#2e911a");
        Drawing.drawPath(follower.getCurrentPath(),"#2e911a");
        Drawing.sendPacket();
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
    public void pause(double time, double x, double y, double degrees) {
        ElapsedTime myTimer = new ElapsedTime();
        myTimer.reset();
        while (myTimer.seconds() < time) {
            telemetry.addData("Elapsed Time", myTimer.seconds());
            telemetry.update();

            follower.holdPoint(new BezierPoint(new Point(x, y, Point.CARTESIAN)), Math.toRadians(degrees));
            // sleep(50);
        }
    }
}