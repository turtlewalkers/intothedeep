package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.teleop.PIDF.d;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.f;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.i;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.p;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.ticks_in_degrees;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOMINIT;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOM_SCAN_SUB;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OUTTAKECLAW1;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OUTTAKECLAW2;
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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;
import org.firstinspires.ftc.teamcode.teleop.Teleop;

@Config
@Autonomous
public class Specimen extends LinearOpMode {
    private static int SLIDE_HEIGHT;
    TurtleRobot robot = new TurtleRobot(this);
    private ElapsedTime timeElapsed;

    private Follower follower;
    private PathChain paths;
    private Path path1, path2, path3, path4, path5, path6, path7;
    private Path path8, path9, path10;
    private PIDController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        robot.init(hardwareMap);
        robot.topLeft.setPosition(TOPINIT);
        robot.bottomRight.setPosition(BOTTOMINIT);
        robot.bottomLeft.setPosition(BOTTOMINIT);
        robot.smartServo.setPosition(SPEC_DROP_SMART);
        robot.arm.setPosition(SPEC_DROP_ARM);
        robot.outtake.setPosition(OUTTAKECLAW2);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7, 57, 0));
        timeElapsed = new ElapsedTime();

        path1 = new Path(new BezierCurve(new Point(7, 57, Point.CARTESIAN), new Point(32, 68, Point.CARTESIAN)));
        path1.setConstantHeadingInterpolation(0);

        path2 = new Path(new BezierCurve(new Point(32, 68, Point.CARTESIAN), new Point(5, 25, Point.CARTESIAN), new Point(55, 37, Point.CARTESIAN)));
        path2.setConstantHeadingInterpolation(0);

        paths = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(63, 22, Point.CARTESIAN), new Point(20, 22, Point.CARTESIAN))) // path3
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(20, 22, Point.CARTESIAN), new Point(70, 43, Point.CARTESIAN), new Point(60, 17, Point.CARTESIAN))) // path4
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(60, 17, Point.CARTESIAN), new Point(20, 17, Point.CARTESIAN))) // path5
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(20, 15, Point.CARTESIAN), new Point(70, 35, Point.CARTESIAN), new Point(60, 10, Point.CARTESIAN))) // path6
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(60, 10, Point.CARTESIAN), new Point(20, 7, Point.CARTESIAN))) // path7
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

        path8 = new Path(new BezierCurve(new Point(20, 9, Point.CARTESIAN), new Point(50, 15, Point.CARTESIAN), new Point(10, 37, Point.CARTESIAN)));
        path8.setConstantHeadingInterpolation(0);

        path9 = new Path(new BezierCurve(new Point(7, 37, Point.CARTESIAN), new Point(35, 68, Point.CARTESIAN)));
        path9.setConstantHeadingInterpolation(0);

        path10 = new Path(new BezierCurve(new Point(35, 68, Point.CARTESIAN), new Point(10, 37, Point.CARTESIAN)));
        path10.setConstantHeadingInterpolation(0);
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
        robot.outtake.setPosition(OUTTAKECLAW2);
        robot.topLeft.setPosition(TOP_SCAN_SUB);
        robot.bottomRight.setPosition(BOTTOM_SCAN_SUB);
        robot.bottomLeft.setPosition(BOTTOM_SCAN_SUB);
        followPath(path1);
        SLIDE_HEIGHT = -1300;
        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSlide.setPower(1);
        robot.rightSlide.setPower(1);
        waitForLinearSlide(SLIDE_HEIGHT);
        sleep(750);
        robot.outtake.setPosition(OUTTAKECLAW1);

        followPath(path2);
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
        robot.outtake.setPosition(OUTTAKECLAW1);

        followPath(paths);

        followPath(path8);
        for (int i = 0; i < 3; ++i) {
            sleep(500);
            robot.outtake.setPosition(OUTTAKECLAW2);
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
            SLIDE_HEIGHT = -1300;
            robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftSlide.setPower(1);
            robot.rightSlide.setPower(1);
            waitForLinearSlide(SLIDE_HEIGHT);
            sleep(750);
            robot.outtake.setPosition(OUTTAKECLAW1);
            robot.smartServo.setPosition(SPEC_PICK_SMARTSERVO);
            robot.arm.setPosition(SPEC_PICK_ARMSERVO);
            robot.outtake.setPosition(OUTTAKECLAW1);
            SLIDE_HEIGHT = 0;
            robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftSlide.setPower(1);
            robot.rightSlide.setPower(1);
            waitForLinearSlide(SLIDE_HEIGHT);
            followPath(path10);

            waitForStart();
            while (opModeIsActive()) {
                telemetry.addData("Holding Point", "true");
                telemetry.update();
                //follower.holdPoint(new BezierPoint(new Point(0, 0, Point.CARTESIAN)), Math.toRadians(0));
                UpdatePathAndTelemetry();
            }

        }
    }
    public void followPath(PathChain path) {
        follower.followPath(path);
        while (follower.isBusy())
            UpdatePathAndTelemetry();
    }
    public void followPath(Path path) {
        follower.followPath(path);
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