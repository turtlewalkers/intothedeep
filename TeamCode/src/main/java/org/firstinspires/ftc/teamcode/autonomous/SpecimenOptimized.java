package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.teleop.PIDF.d;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.f;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.i;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.p;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.ticks_in_degrees;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.BOTTOMINIT;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.OUTTAKECLAW2;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_DROP_ARM;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_DROP_SMART;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.SPEC_PICK_ARMSERVO;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TOPINIT;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
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
public class SpecimenOptimized extends LinearOpMode {
    private static int SLIDE_HEIGHT;
    TurtleRobot robot = new TurtleRobot(this);
    private ElapsedTime timeElapsed;

    private Follower follower;
    private PathChain paths;
    private Path path1, path2, path3, path4, path5, path6, path7;
    private PIDController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
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

        paths = follower.pathBuilder()
                .addTemporalCallback(0.000002, () -> {
                    SLIDE_HEIGHT = 0;
                })
                .addTemporalCallback(0.5, () -> {
                    SLIDE_HEIGHT = -700;
                    robot.smartServo.setPosition(SPEC_DROP_SMART);
                    robot.arm.setPosition(SPEC_DROP_ARM);
                    robot.outtake.setPosition(OUTTAKECLAW2);
                })
                .addPath(new BezierCurve(new Point(7, 57, Point.CARTESIAN), new Point(27, 68, Point.CARTESIAN))) // path1
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(27, 68, Point.CARTESIAN), new Point(5, 25, Point.CARTESIAN), new Point(55, 37, Point.CARTESIAN))) // path2
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(63, 25, Point.CARTESIAN), new Point(20, 25, Point.CARTESIAN))) // path3
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(20, 25, Point.CARTESIAN), new Point(70, 43, Point.CARTESIAN), new Point(60, 17, Point.CARTESIAN))) // path4
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(60, 17, Point.CARTESIAN), new Point(20, 17, Point.CARTESIAN))) // path5
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(20, 15, Point.CARTESIAN), new Point(70, 35, Point.CARTESIAN), new Point(60, 7, Point.CARTESIAN))) // path6
                .setConstantHeadingInterpolation(0)

                .addPath(new BezierCurve(new Point(60, 7, Point.CARTESIAN), new Point(20, 7, Point.CARTESIAN))) // path7
                .setConstantHeadingInterpolation(0)

                .build();

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

        follower.followPath(paths);


        while (opModeIsActive()) {
            follower.update();
            controller.setPID(p, i, d);
            int linearSlidePosition = robot.leftSlide.getCurrentPosition();
            double pid = controller.calculate(linearSlidePosition, SLIDE_HEIGHT);
            double ff = Math.toRadians(SLIDE_HEIGHT / ticks_in_degrees) * f;
            double power = pid + ff;
            robot.leftSlide.setPower(power);
            robot.rightSlide.setPower(power);

            telemetry.addData("pos", linearSlidePosition);
            telemetry.addData("target", SLIDE_HEIGHT);
            telemetry.update();
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