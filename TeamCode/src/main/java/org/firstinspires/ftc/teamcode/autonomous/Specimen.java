package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.teleop.PIDF.d;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.i;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.p;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOMINIT;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOM_TRANSFER;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OFSETLEFT;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OFSETRIGHT;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OUTTAKEOPEN;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OUTTAKECLOSE;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_DROP_ARM;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_DROP_SMART;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_PICK_ARMSERVO;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_PICK_SMARTSERVO;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOPINIT;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOP_SCAN_SUB;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Config
@Autonomous
public class Specimen extends LinearOpMode {
    private static int SLIDE_HEIGHT;
    TurtleRobot robot = new TurtleRobot(this);
    private ElapsedTime timeElapsed;

    private Follower follower;
    private PathChain paths, path10, path8;
    private Path path1, path2, path3, path4, path5, path6, path7, path11;
    private Path path9;
    private PIDController controller;

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
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7, 57, 0));
        timeElapsed = new ElapsedTime();

        path1 = new Path(new BezierCurve(new Point(7, 57, Point.CARTESIAN), new Point(35, 68, Point.CARTESIAN)));
        path1.setConstantHeadingInterpolation(0);

//        path2 = new Path(new BezierCurve(new Point(32, 68, Point.CARTESIAN), new Point(5, 25, Point.CARTESIAN), new Point(55, 36, Point.CARTESIAN)));
//        path2.setConstantHeadingInterpolation(0);

        paths = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(32, 68, Point.CARTESIAN),
                        new Point(14, 25, Point.CARTESIAN),
                        new Point(45, 36, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(63, 25, Point.CARTESIAN),
                        new Point(26, 25, Point.CARTESIAN))) // path3
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(26, 25, Point.CARTESIAN),
                        new Point(70, 25, Point.CARTESIAN),
                        new Point(50, 17, Point.CARTESIAN))) // path4
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(50, 17, Point.CARTESIAN),
                        new Point(26, 17, Point.CARTESIAN))) // path5
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(26, 15, Point.CARTESIAN),
                        new Point(70, 14, Point.CARTESIAN),
                        new Point(50, 8, Point.CARTESIAN))) // path6
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(60, 10, Point.CARTESIAN),
                        new Point(26, 10, Point.CARTESIAN))) // path7
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(20, 10, Point.CARTESIAN),
                        new Point(30, 42, Point.CARTESIAN),
                        new Point(8, 36, Point.CARTESIAN)))
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
                .addPath(new BezierCurve(new Point(20, 9, Point.CARTESIAN), new Point(40, 20, Point.CARTESIAN), new Point(8, 36, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)
                .build();

        path9 = new Path(new BezierCurve(new Point(7, 37, Point.CARTESIAN), new Point(36, 68, Point.CARTESIAN)));
        path9.setConstantHeadingInterpolation(0);

//        path10 = new Path(new BezierCurve(new Point(35, 68, Point.CARTESIAN), new Point(12, 35, Point.CARTESIAN)));
//        path10.setConstantHeadingInterpolation(0);

        path10 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(35, 68, Point.CARTESIAN),
                                new Point(19, 30, Point.CARTESIAN),
                                new Point(23, 36, Point.CARTESIAN),
                                new Point(9, 36, Point.CARTESIAN)))
                        .setConstantHeadingInterpolation(0)
                                .build();

        waitForStart();

        SLIDE_HEIGHT = -700;
        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSlide.setPower(1);
        robot.rightSlide.setPower(1);
        waitForLinearSlide(SLIDE_HEIGHT);
        robot.smartServo.setPosition(SPEC_DROP_SMART);
        robot.arm.setPosition(SPEC_DROP_ARM);
        robot.outtake.setPosition(OUTTAKECLOSE);
        robot.topLeft.setPosition(TOP_SCAN_SUB);
        robot.bottomRight.setPosition(BOTTOM_TRANSFER + OFSETRIGHT);
        robot.bottomLeft.setPosition(BOTTOM_TRANSFER + OFSETLEFT);
        followPath(path1);
        SLIDE_HEIGHT = -1350;
        robot.smartServo.setPosition(SPEC_DROP_SMART+0.2);
        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSlide.setPower(1);
        robot.rightSlide.setPower(1);
        waitForLinearSlide(SLIDE_HEIGHT);
        sleep(500);
        robot.outtake.setPosition(OUTTAKEOPEN);
        robot.smartServo.setPosition(SPEC_PICK_SMARTSERVO);
        robot.arm.setPosition(SPEC_PICK_ARMSERVO);
        robot.outtake.setPosition(OUTTAKEOPEN);
        SLIDE_HEIGHT = 0;
        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSlide.setPower(1);
        robot.rightSlide.setPower(1);
        waitForLinearSlide(SLIDE_HEIGHT);
//        followPath(path2);

        followPath(paths);

//        followPath(path8);
//        follower.setMaxPower(0.9);
        for (int i = 0; i < 4; ++i) {
            robot.outtake.setPosition(OUTTAKECLOSE);
//            sleep(100);
            SLIDE_HEIGHT = -700;
            robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftSlide.setPower(1);
            robot.rightSlide.setPower(1);
            waitForLinearSlide(SLIDE_HEIGHT);
            robot.smartServo.setPosition(SPEC_DROP_SMART);
            robot.arm.setPosition(SPEC_DROP_ARM);
            followPath(path9);
//            follower.setMaxPower(1);
            SLIDE_HEIGHT = -1350;
            robot.smartServo.setPosition(SPEC_DROP_SMART+0.2);
            robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftSlide.setPower(1);
            robot.rightSlide.setPower(1);
            waitForLinearSlide(SLIDE_HEIGHT);
            sleep(500);
            robot.outtake.setPosition(OUTTAKEOPEN);
            robot.smartServo.setPosition(SPEC_PICK_SMARTSERVO);
            robot.arm.setPosition(SPEC_PICK_ARMSERVO);
            SLIDE_HEIGHT = 0;
            robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftSlide.setPower(1);
            robot.rightSlide.setPower(1);
            waitForLinearSlide(SLIDE_HEIGHT);
            followPath(path10);
//            follower.setMaxPower(0.9);
        }
    }
    public void followPath(PathChain path) {
        follower.followPath(path);
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