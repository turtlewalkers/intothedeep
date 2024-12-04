package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOMINIT;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_PICK_ARMSERVO;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOPINIT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
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

    private Path path1, path2, path3, path4, path5, path6, path7;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.topLeft.setPosition(TOPINIT);
        robot.bottomRight.setPosition(BOTTOMINIT);
        robot.bottomLeft.setPosition(BOTTOMINIT);
        robot.smartServo.setPosition(0.3);
        robot.arm.setPosition(SPEC_PICK_ARMSERVO);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7, 57, 0));
        timeElapsed = new ElapsedTime();

        path1 = new Path(new BezierCurve(new Point(7,57,Point.CARTESIAN), new Point(33, 68, Point.CARTESIAN)));
        path1.setConstantHeadingInterpolation(0);
        path2 = new Path(new BezierCurve(new Point(47,68,Point.CARTESIAN), new Point(29,46,Point.CARTESIAN), new Point(63,25,Point.CARTESIAN)));
        path2.setConstantHeadingInterpolation(Math.PI / 2);
        path3 = new Path(new BezierCurve(new Point(63,25,Point.CARTESIAN), new Point(10, 25, Point.CARTESIAN)));
        path3.setConstantHeadingInterpolation(Math.PI / 2);

        path4 = new Path(new BezierCurve(new Point(10,25,Point.CARTESIAN), new Point(91, 57, Point.CARTESIAN), new Point(60, 14, Point.CARTESIAN)));
        path4.setConstantHeadingInterpolation(Math.PI / 2);
        path5 = new Path(new BezierCurve(new Point(60,14,Point.CARTESIAN), new Point(10, 14, Point.CARTESIAN)));
        path5.setConstantHeadingInterpolation(Math.PI / 2);
        path6 = new Path(new BezierCurve(new Point(10,6,Point.CARTESIAN), new Point(91, 49, Point.CARTESIAN), new Point(60, 14, Point.CARTESIAN)));
        path6.setConstantHeadingInterpolation(Math.PI / 2);

        path7 = new Path(new BezierCurve(new Point(60,6,Point.CARTESIAN), new Point(10, 6, Point.CARTESIAN)));
        path7.setConstantHeadingInterpolation(Math.PI / 2);


        waitForStart();

        followPath(path1);
        followPath(path2);
        followPath(path3);
        followPath(path4);
        followPath(path5);
        followPath(path6);
        followPath(path7);

        while (opModeIsActive()) {
            telemetry.addData("Holding Point", "true");
            telemetry.update();
            UpdatePathAndTelemetry();
        }

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
}