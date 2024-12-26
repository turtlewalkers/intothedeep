package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.teleop.PIDF.d;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.i;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.p;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOMINIT;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOM_OBSERVE;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOM_SCAN_SUB;
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
public class SpecimenOptimized extends LinearOpMode {
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

        paths = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(7, 57, Point.CARTESIAN),
                        new Point(20, 49, Point.CARTESIAN))) // path3
                .setLinearHeadingInterpolation(0, -Math.PI/6)

                .addPath(new BezierCurve(new Point(20, 49, Point.CARTESIAN),
                        new Point(20, 45, Point.CARTESIAN))) // path4
                .setLinearHeadingInterpolation(-Math.PI/6, -Math.PI*2/3)

                .addPath(new BezierCurve(new Point(20, 45, Point.CARTESIAN),
                        new Point(20, 39, Point.CARTESIAN))) // path3
                .setLinearHeadingInterpolation(0, -Math.PI/6)

                .addPath(new BezierCurve(new Point(20, 39, Point.CARTESIAN),
                        new Point(20, 35, Point.CARTESIAN))) // path4
                .setLinearHeadingInterpolation(-Math.PI/6, -Math.PI*2/3)


//                .addPath(new BezierCurve(new Point(20, 15, Point.CARTESIAN), new Point(70, 35, Point.CARTESIAN), new Point(60, 10, Point.CARTESIAN))) // path6
//                .setConstantHeadingInterpolation(0)
//
//                .addPath(new BezierCurve(new Point(60, 10, Point.CARTESIAN), new Point(20, 7, Point.CARTESIAN))) // path7
//                .setConstantHeadingInterpolation(0)

                .build();


        waitForStart();

        robot.leftHorizontalSlide.setPosition(0.8);
        robot.rightHorizontalSlide.setPosition(0.8);
        robot.topLeft.setPosition(0.62);
        robot.bottomRight.setPosition(BOTTOM_SCAN_SUB - 0.24);
        robot.bottomLeft.setPosition(BOTTOM_SCAN_SUB + 0.24);
        followPath(paths);
    }
    public void followPath(PathChain path) {
        follower.followPath(path, true);
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