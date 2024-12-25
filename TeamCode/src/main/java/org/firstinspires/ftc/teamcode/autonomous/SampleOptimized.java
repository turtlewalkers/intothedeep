package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.teleop.PIDF.d;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.f;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.i;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.p;
import static org.firstinspires.ftc.teamcode.teleop.PIDF.ticks_in_degrees;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;
import org.firstinspires.ftc.teamcode.teleop.Teleop;

public class SampleOptimized extends LinearOpMode {
    private static int SLIDE_HEIGHT;
    TurtleRobot robot = new TurtleRobot(this);
    private ElapsedTime timeElapsed;

    private Follower follower;
    private PathChain paths;
    private PIDController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(10, 104, 0));
        paths = follower.pathBuilder()
                .addTemporalCallback(0.000002, () -> {
                    SLIDE_HEIGHT = 0;
                    robot.leftHorizontalSlide.setPosition(0);
                    robot.rightHorizontalSlide.setPosition(0);
                })
                .addPath(new BezierCurve(new Point(17,126,Point.CARTESIAN), new Point(21.5, 123.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(-Math.PI / 4)
                .addTemporalCallback(0.3, () -> {
                    SLIDE_HEIGHT = -2350;
                    robot.smartServo.setPosition(Teleop.BASKET_SMARTSERVO);
                    robot.arm.setPosition(Teleop.BASKET_ARMSERVO);
                    sleep(500);
                })
                .addPath(new BezierCurve(new Point(15,120,Point.CARTESIAN), new Point(16,125,Point.CARTESIAN)))
                .setConstantHeadingInterpolation(-Math.PI / 4)
                .addTemporalCallback(3.5, () -> {
                    robot.outtake.setPosition(0);
                    //robot.topRight.setPosition(Teleop.TOP_OBSERVE);
                    robot.topLeft.setPosition(Teleop.TOP_OBSERVE);
                    robot.intake.setPosition(Teleop.OPENINTAKE);
                    robot.bottomRight.setPosition(Teleop.BOTTOM_OBSERVE);
                    robot.bottomLeft.setPosition(Teleop.BOTTOM_OBSERVE);
                    sleep(100);
                })
                .addPath(new BezierCurve(new Point(17,126,Point.CARTESIAN), new Point(21.5, 123.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(0)
                .addTemporalCallback(3.7, () -> {
                    robot.leftHorizontalSlide.setPosition(0);
                    robot.rightHorizontalSlide.setPosition(0);
                    robot.smartServo.setPosition(Teleop.TX_PICKUP_SMARTSERVO);
                    robot.arm.setPosition(Teleop.TX_PICKUP_ARMSERVO+0.05);
                    robot.intake.setPosition(Teleop.OPENINTAKE);
                    sleep(500);
                })
                .addTemporalCallback(4.2, () -> {
                    robot.leftHorizontalSlide.setPosition(0.5);
                    robot.rightHorizontalSlide.setPosition(0.5);
                    sleep(300);
                    //robot.topRight.setPosition(Teleop.TOP_PICK);
                    robot.topLeft.setPosition(Teleop.TOP_PICK);
                    robot.bottomRight.setPosition(Teleop.BOTTOM_PICK);
                    robot.bottomLeft.setPosition(Teleop.BOTTOM_PICK);
                    sleep(100);
                    robot.intake.setPosition(Teleop.CLOSEINTAKE);
                    sleep(100);
                    robot.leftHorizontalSlide.setPosition(0);
                    robot.rightHorizontalSlide.setPosition(0);
                    //robot.topRight.setPosition(Teleop.TOP_TRANSFER);
                    robot.topLeft.setPosition(Teleop.TOP_TRANSFER);
                    robot.bottomRight.setPosition(Teleop.BOTTOM_TRANSFER);
                    robot.bottomLeft.setPosition(Teleop.BOTTOM_TRANSFER);
                    sleep(500);
                    robot.intake.setPosition(Teleop.OPENINTAKE);
                })
                .addPath(new BezierCurve(new Point(23,125, Point.CARTESIAN), new Point(18,124, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(-Math.PI / 4)
                .addTemporalCallback(5.2, () -> {
                    sleep(500);
                    robot.arm.setPosition(Teleop.TX_PICKUP_ARMSERVO);
                    sleep(500);
                    robot.outtake.setPosition(Teleop.OUTTAKEOPEN);
                    sleep(500);
                    SLIDE_HEIGHT = -2350;
                    robot.leftHorizontalSlide.setPosition(0);
                    robot.rightHorizontalSlide.setPosition(0);
                    robot.smartServo.setPosition(Teleop.BASKET_SMARTSERVO);
                    robot.arm.setPosition(Teleop.BASKET_ARMSERVO);
                })
                .build();

        waitForStart();

        follower.followPath(paths);
        while (opModeIsActive()) {
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
}
