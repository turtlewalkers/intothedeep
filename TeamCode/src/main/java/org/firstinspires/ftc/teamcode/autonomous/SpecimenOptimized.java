package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.teleop.PIDF.d;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.i;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.p;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOMINIT;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOM_OBSERVE;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOM_PICK;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOM_SCAN_SUB;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOM_TRANSFER;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.CLOSEINTAKE;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OPENINTAKE;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OUTTAKECLOSE;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OUTTAKEOPEN;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_DROP_SMART;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_PICK_ARMSERVO;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOPINIT;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOP_OBSERVE;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOP_PICK;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOP_SCAN_SUB;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOP_TRANSFER;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TX_PICKUP_ARMSERVO;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TX_PICKUP_SMARTSERVO;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Config
@Autonomous
public class SpecimenOptimized extends LinearOpMode {
    private static int SLIDE_HEIGHT;
    TurtleRobot robot = new TurtleRobot(this);
    private ElapsedTime timeElapsed;

    private Follower follower;
    private PathChain paths, path10, path8;
    private Path sample1, sample2, sample3, sample4, path5, path6, path7, path11;
    private Path path9;
    private PIDController controller;

    public static int sample1x = 18;
    public static int sample1y = 29;
    public static int sample2x = 18;
    public static int sample2y = 19;
    public static int sample3x = 20;
    public static int sample3y = 16;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        robot.init(hardwareMap);
        robot.topLeft.setPosition(TOPINIT);
        robot.bottomRight.setPosition(BOTTOMINIT);
        robot.bottomLeft.setPosition(BOTTOMINIT);
//        robot.smartServo.setPosition(SPEC_DROP_SMART);
//        robot.arm.setPosition(SPEC_DROP_ARM);
        robot.outtake.setPosition(OUTTAKECLOSE);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7, 57, 0));
        timeElapsed = new ElapsedTime();

        paths = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(32, 68, Point.CARTESIAN),
                        new Point(14, 25, Point.CARTESIAN),
                        new Point(45, 36, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(63, 25, Point.CARTESIAN),
                        new Point(26, 25, Point.CARTESIAN))) // sample3
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(26, 25, Point.CARTESIAN),
                        new Point(70, 25, Point.CARTESIAN),
                        new Point(50, 17, Point.CARTESIAN))) // sample4
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(50, 17, Point.CARTESIAN),
                        new Point(26, 17, Point.CARTESIAN))) // path5
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(26, 15, Point.CARTESIAN),
                        new Point(70, 14, Point.CARTESIAN),
                        new Point(50, 10, Point.CARTESIAN))) // path6
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(60, 10, Point.CARTESIAN),
                        new Point(26, 10, Point.CARTESIAN))) // path7
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(20, 10, Point.CARTESIAN),
                        new Point(30, 42, Point.CARTESIAN),
                        new Point(8, 36, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)

                .build();
        sample1 = new Path(new BezierCurve(new Point(7, 57, Point.CARTESIAN),
                new Point(sample1x, sample1y, Point.CARTESIAN)));
        sample1.setLinearHeadingInterpolation(0, 0);

        sample2 = new Path(new BezierCurve(new Point(sample1x, sample1y, Point.CARTESIAN),
                new Point(sample2x, sample2y, Point.CARTESIAN)));
        sample2.setConstantHeadingInterpolation(0);

        sample3 = new Path(new BezierCurve(new Point(sample2x, sample2y, Point.CARTESIAN),
                new Point(sample3x, sample3y, Point.CARTESIAN)));
        sample3.setConstantHeadingInterpolation(-Math.PI/6);

        sample4 = new Path(new BezierCurve(new Point(20, 10, Point.CARTESIAN),
                new Point(30, 42, Point.CARTESIAN),
                new Point(8, 36, Point.CARTESIAN)));
        sample4.setConstantHeadingInterpolation(0);

        path5 = new Path(new BezierCurve(new Point(20, 39, Point.CARTESIAN),
                new Point(7, 57, Point.CARTESIAN)));
        path5.setConstantHeadingInterpolation(0);

        waitForStart();

        followPath(sample1);

        robot.leftHorizontalSlide.setPosition(0.8);
        robot.rightHorizontalSlide.setPosition(0.8);
        robot.topLeft.setPosition(TOP_OBSERVE);
        robot.bottomRight.setPosition(BOTTOM_OBSERVE);
        robot.bottomLeft.setPosition(BOTTOM_OBSERVE);
        robot.intake.setPosition(OPENINTAKE);
        robot.outtake.setPosition(OUTTAKEOPEN);
        robot.arm.setPosition(TX_PICKUP_ARMSERVO+0.1);
        robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
        sleep(400);
        robot.topLeft.setPosition(TOP_PICK);
        robot.bottomRight.setPosition(BOTTOM_PICK);
        robot.bottomLeft.setPosition(BOTTOM_PICK);
        robot.outtake.setPosition(OUTTAKEOPEN);
        sleep(200);
        robot.intake.setPosition(CLOSEINTAKE);
        sleep(100);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        robot.topLeft.setPosition(TOP_TRANSFER);
        robot.bottomRight.setPosition(BOTTOM_TRANSFER);
        robot.bottomLeft.setPosition(BOTTOM_TRANSFER);
        sleep(500);
        robot.arm.setPosition(TX_PICKUP_ARMSERVO);
        robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
        sleep(200);
        robot.intake.setPosition(OPENINTAKE);
        sleep(250);
        robot.outtake.setPosition(OUTTAKECLOSE);
        sleep(250);
        robot.arm.setPosition(SPEC_PICK_ARMSERVO);
        robot.smartServo.setPosition(SPEC_DROP_SMART);

        followPath(sample2);

        robot.outtake.setPosition(OUTTAKEOPEN);
        sleep(100);
        robot.leftHorizontalSlide.setPosition(0.8);
        robot.rightHorizontalSlide.setPosition(0.8);
        robot.topLeft.setPosition(TOP_OBSERVE);
        robot.bottomRight.setPosition(BOTTOM_OBSERVE);
        robot.bottomLeft.setPosition(BOTTOM_OBSERVE);
        robot.intake.setPosition(OPENINTAKE);
        robot.arm.setPosition(TX_PICKUP_ARMSERVO+0.1);
        robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
        sleep(400);
        robot.topLeft.setPosition(TOP_PICK);
        robot.bottomRight.setPosition(BOTTOM_PICK);
        robot.bottomLeft.setPosition(BOTTOM_PICK);
        robot.outtake.setPosition(OUTTAKEOPEN);
        sleep(200);
        robot.intake.setPosition(CLOSEINTAKE);
        sleep(100);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        robot.topLeft.setPosition(TOP_TRANSFER);
        robot.bottomRight.setPosition(BOTTOM_TRANSFER);
        robot.bottomLeft.setPosition(BOTTOM_TRANSFER);
        sleep(500);
        robot.arm.setPosition(TX_PICKUP_ARMSERVO);
        robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
        sleep(200);
        robot.intake.setPosition(OPENINTAKE);
        sleep(250);
        robot.outtake.setPosition(OUTTAKECLOSE);
        sleep(250);
        robot.arm.setPosition(SPEC_PICK_ARMSERVO);
        robot.smartServo.setPosition(SPEC_DROP_SMART);

        followPath(sample3);

        robot.outtake.setPosition(OUTTAKEOPEN);
        sleep(100);
        robot.leftHorizontalSlide.setPosition(0.8);
        robot.rightHorizontalSlide.setPosition(0.8);
        robot.topLeft.setPosition(TOP_OBSERVE);
        robot.bottomRight.setPosition(BOTTOM_OBSERVE+0.1);
        robot.bottomLeft.setPosition(BOTTOM_OBSERVE-0.1);
        robot.intake.setPosition(OPENINTAKE);
        robot.outtake.setPosition(OUTTAKEOPEN);
        robot.arm.setPosition(TX_PICKUP_ARMSERVO+0.1);
        robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
        sleep(400);
        robot.topLeft.setPosition(TOP_PICK);
        robot.bottomRight.setPosition(BOTTOM_PICK);
        robot.bottomLeft.setPosition(BOTTOM_PICK);
        robot.outtake.setPosition(OUTTAKEOPEN);
        sleep(200);
        robot.intake.setPosition(CLOSEINTAKE);
        sleep(100);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        robot.topLeft.setPosition(TOP_TRANSFER);
        robot.bottomRight.setPosition(BOTTOM_TRANSFER);
        robot.bottomLeft.setPosition(BOTTOM_TRANSFER);
        sleep(500);
        robot.arm.setPosition(TX_PICKUP_ARMSERVO);
        robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
        sleep(200);
        robot.intake.setPosition(OPENINTAKE);
        sleep(250);
        robot.outtake.setPosition(OUTTAKECLOSE);
        sleep(250);
        robot.arm.setPosition(SPEC_PICK_ARMSERVO);
        robot.smartServo.setPosition(SPEC_DROP_SMART);
        sleep(500);
        robot.outtake.setPosition(OUTTAKEOPEN);
        followPath(sample4);
    }
    public void followPath(PathChain path) {
        follower.followPath(path, true);
        while (follower.isBusy())
            UpdatePathAndTelemetry();
    }
    public void followPath(Path path) {
        follower.followPath(path, true);
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