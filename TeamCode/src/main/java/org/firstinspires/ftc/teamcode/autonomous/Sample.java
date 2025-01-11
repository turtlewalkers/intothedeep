package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.teleop.Teleop.TX_PICKUP_ARMSERVO;
import static org.firstinspires.ftc.teamcode.teleop.Teleop.TX_PICKUP_SMARTSERVO;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.teleop.Teleop;

@Config
@Autonomous
public class Sample extends LinearOpMode {
    private static int SLIDE_HEIGHT;
    TurtleRobot robot = new TurtleRobot(this);
    private ElapsedTime timeElapsed;

    private Follower follower;

    private Path path1, path2, path3, path4, path5, path6;

    private PIDController controller;

    private Path sample1, sample2, sample3, sample4;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.topLeft.setPosition(0);
        //robot.topRight.setPosition(0);
        robot.bottomRight.setPosition(0.1);
        robot.bottomLeft.setPosition(0.1);
        robot.outtake.setPosition(1);
        robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightHorizontalSlide.setPosition(0);
        robot.leftHorizontalSlide.setPosition(0);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(10, 104, 0));
        timeElapsed = new ElapsedTime();

        // go to basket
        path1 = new Path(new BezierCurve(new Point(10,104,Point.CARTESIAN), new Point(17,120,Point.CARTESIAN)));
        path1.setConstantHeadingInterpolation(-Math.PI / 4);
        // go to position for dropping sample
        path2 = new Path(new BezierCurve(new Point(15,120,Point.CARTESIAN), new Point(16,125,Point.CARTESIAN)));
        path2.setConstantHeadingInterpolation(-Math.PI / 4);
        // first one
        path3 = new Path(new BezierCurve(new Point(15,126,Point.CARTESIAN), new Point(18, 123, Point.CARTESIAN)));
        path3.setConstantHeadingInterpolation(0);
        path4 = new Path(new BezierCurve(new Point(23,125, Point.CARTESIAN), new Point(18,124, Point.CARTESIAN)));
        path4.setConstantHeadingInterpolation(-Math.PI / 4);
        // second one
        path5 = new Path(new BezierCurve(new Point(17, 126, Point.CARTESIAN), new Point(19, 128, Point.CARTESIAN)));
        path5.setConstantHeadingInterpolation(0);
        // third one
        path6 = new Path(new BezierCurve(new Point(20, 129, Point.CARTESIAN), new Point(25, 133, Point.CARTESIAN)));
        path6.setConstantHeadingInterpolation(Math.PI / 8);

        waitForStart();

        // drop the first one
        followPath(path1);
        int linearSlideTargetHeight = -2350;
//        SLIDE_HEIGHT = -2350;
//        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftSlide.setPower(1);
//        robot.rightSlide.setPower(1);
//        robot.leftHorizontalSlide.setPosition(0);
//        robot.rightHorizontalSlide.setPosition(0);
//        waitForLinearSlide(linearSlideTargetHeight);
        robot.smartServo.setPosition(Teleop.BASKET_SMARTSERVO);
        robot.arm.setPosition(Teleop.BASKET_ARMSERVO);

        sleep(1000);

        followPath(path2);
        sleep(250);
        robot.outtake.setPosition(0);

        // second sample
        //robot.topRight.setPosition(Teleop.TOP_OBSERVE);
        robot.topLeft.setPosition(Teleop.TOP_OBSERVE);
        robot.intake.setPosition(Teleop.OPENINTAKE);
        telemetry.addData("right", robot.bottomRight.getPosition());
        telemetry.addData("left", robot.bottomLeft.getPosition());
        telemetry.update();
        robot.bottomRight.setPosition(Teleop.BOTTOM_OBSERVE);
        robot.bottomLeft.setPosition(Teleop.BOTTOM_OBSERVE);

        sleep(500);
        followPath(path3);

        linearSlideTargetHeight = 0;
//        SLIDE_HEIGHT = 0;
//        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftSlide.setPower(1);
//        robot.rightSlide.setPower(1);
//        waitForLinearSlide(linearSlideTargetHeight);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
        robot.arm.setPosition(TX_PICKUP_ARMSERVO+0.1);
        robot.intake.setPosition(Teleop.OPENINTAKE);
//        sleep(500);

        robot.leftHorizontalSlide.setPosition(1);
        robot.rightHorizontalSlide.setPosition(1);
        sleep(500);
        //robot.topRight.setPosition(Teleop.TOP_PICK);
        robot.topLeft.setPosition(Teleop.TOP_PICK);
        telemetry.addData("right", robot.bottomRight.getPosition());
        telemetry.addData("left", robot.bottomLeft.getPosition());
        telemetry.update();
        robot.bottomRight.setPosition(Teleop.BOTTOM_PICK);
        robot.bottomLeft.setPosition(Teleop.BOTTOM_PICK);
        sleep(250);
        robot.intake.setPosition(Teleop.CLOSEINTAKE);


        sleep(500);

        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        //robot.topRight.setPosition(Teleop.TOP_TRANSFER);
        telemetry.addData("right", robot.bottomRight.getPosition());
        telemetry.addData("left", robot.bottomLeft.getPosition());
        telemetry.update();
        robot.bottomRight.setPosition(Teleop.BOTTOM_TRANSFER);
        robot.bottomLeft.setPosition(Teleop.BOTTOM_TRANSFER);
        sleep(500);
        robot.topLeft.setPosition(Teleop.TOP_TRANSFER);
        followPath(path4);
        sleep(200);
        robot.arm.setPosition(TX_PICKUP_ARMSERVO);
        robot.intake.setPosition(Teleop.OPENINTAKE);
        sleep(200);
        robot.outtake.setPosition(Teleop.OUTTAKECLOSE);
        sleep(500);
//        linearSlideTargetHeight = -2350;
//        SLIDE_HEIGHT = -2350;
//        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftSlide.setPower(1);
//        robot.rightSlide.setPower(1);
//        robot.leftHorizontalSlide.setPosition(0);
//        robot.rightHorizontalSlide.setPosition(0);
//        waitForLinearSlide(linearSlideTargetHeight);
        robot.smartServo.setPosition(Teleop.BASKET_SMARTSERVO);
        robot.arm.setPosition(Teleop.BASKET_ARMSERVO);
        sleep(750);
        followPath(path2);
        sleep(250);
        robot.outtake.setPosition(0);

        // the third one
        //robot.topRight.setPosition(Teleop.TOP_OBSERVE);
        telemetry.addData("right", robot.bottomRight.getPosition());
        telemetry.addData("left", robot.bottomLeft.getPosition());
        telemetry.update();
        robot.bottomRight.setPosition(Teleop.BOTTOM_OBSERVE);
        robot.bottomLeft.setPosition(Teleop.BOTTOM_OBSERVE);
        sleep(50);
        robot.topLeft.setPosition(Teleop.TOP_OBSERVE);
        robot.intake.setPosition(Teleop.OPENINTAKE);

        sleep(500);
        followPath(path5);
        sleep(100);

//        linearSlideTargetHeight = 0;
//        SLIDE_HEIGHT = 0;
//        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftSlide.setPower(1);
//        robot.rightSlide.setPower(1);
//        waitForLinearSlide(linearSlideTargetHeight);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
        robot.arm.setPosition(TX_PICKUP_ARMSERVO+0.1);

//        sleep(1000);
//
        robot.leftHorizontalSlide.setPosition(1);
        robot.rightHorizontalSlide.setPosition(1);
        sleep(500);
        //robot.topRight.setPosition(Teleop.TOP_PICK);
        telemetry.addData("right", robot.bottomRight.getPosition());
        telemetry.addData("left", robot.bottomLeft.getPosition());
        telemetry.update();
        robot.bottomRight.setPosition(Teleop.BOTTOM_PICK);
        robot.bottomLeft.setPosition(Teleop.BOTTOM_PICK);
        robot.topLeft.setPosition(Teleop.TOP_PICK);
        sleep(500);
        robot.intake.setPosition(Teleop.CLOSEINTAKE);

        sleep(250);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        //robot.topRight.setPosition(Teleop.TOP_TRANSFER);
        telemetry.addData("right", robot.bottomRight.getPosition());
        telemetry.addData("left", robot.bottomLeft.getPosition());
        telemetry.update();
        robot.bottomRight.setPosition(Teleop.BOTTOM_TRANSFER);
        robot.bottomLeft.setPosition(Teleop.BOTTOM_TRANSFER);
        sleep(500);
        robot.topLeft.setPosition(Teleop.TOP_TRANSFER);
        followPath(path4);
        sleep(500);
        robot.intake.setPosition(Teleop.OPENINTAKE);
        robot.arm.setPosition(TX_PICKUP_ARMSERVO);
        sleep(500);
        robot.outtake.setPosition(Teleop.OUTTAKECLOSE);
        sleep(250);

//        linearSlideTargetHeight = -2350;
//        SLIDE_HEIGHT = -2350;
//        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftSlide.setPower(1);
//        robot.rightSlide.setPower(1);
//        robot.leftHorizontalSlide.setPosition(0);
//        robot.rightHorizontalSlide.setPosition(0);
//        waitForLinearSlide(linearSlideTargetHeight);
        robot.smartServo.setPosition(Teleop.BASKET_SMARTSERVO);
        robot.arm.setPosition(Teleop.BASKET_ARMSERVO);
        sleep(500);
        followPath(path2);
        sleep(250);
        robot.outtake.setPosition(0);

        // the fourth one
        //robot.topRight.setPosition(Teleop.TOP_OBSERVE);
        telemetry.addData("right", robot.bottomRight.getPosition());
        telemetry.addData("left", robot.bottomLeft.getPosition());
        telemetry.update();
        robot.bottomRight.setPosition(Teleop.BOTTOM_OBSERVE);
        robot.bottomLeft.setPosition(Teleop.BOTTOM_OBSERVE);
        sleep(50);
        robot.topLeft.setPosition(Teleop.TOP_OBSERVE);
        robot.intake.setPosition(Teleop.OPENINTAKE);

        sleep(500);
        followPath(path6);
        sleep(100);

//        linearSlideTargetHeight = 0;
//        SLIDE_HEIGHT = 0;
//        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftSlide.setPower(1);
//        robot.rightSlide.setPower(1);
//        waitForLinearSlide(linearSlideTargetHeight);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
//        waitForLinearSlide(linearSlideTargetHeight);
        robot.smartServo.setPosition(TX_PICKUP_SMARTSERVO);
        robot.arm.setPosition(TX_PICKUP_ARMSERVO+0.1);

//        sleep(1000);

        robot.leftHorizontalSlide.setPosition(0.5);
        robot.rightHorizontalSlide.setPosition(0.5);
        sleep(500);
        //robot.topRight.setPosition(Teleop.TOP_PICK);
        telemetry.addData("right", robot.bottomRight.getPosition());
        telemetry.addData("left", robot.bottomLeft.getPosition());
        telemetry.update();
        robot.bottomRight.setPosition(Teleop.BOTTOM_PICK);
        robot.bottomLeft.setPosition(Teleop.BOTTOM_PICK);
        robot.topLeft.setPosition(Teleop.TOP_PICK);
        sleep(200);
        robot.intake.setPosition(Teleop.CLOSEINTAKE);

        sleep(250);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        //robot.topRight.setPosition(Teleop.TOP_TRANSFER);
        telemetry.addData("right", robot.bottomRight.getPosition());
        telemetry.addData("left", robot.bottomLeft.getPosition());
        telemetry.update();
        robot.bottomRight.setPosition(Teleop.BOTTOM_TRANSFER);
        robot.bottomLeft.setPosition(Teleop.BOTTOM_TRANSFER);
        robot.topLeft.setPosition(Teleop.TOP_TRANSFER);
        followPath(path4);
        sleep(500);
        robot.intake.setPosition(Teleop.OPENINTAKE);
        robot.arm.setPosition(TX_PICKUP_ARMSERVO);
        sleep(250);
        robot.outtake.setPosition(Teleop.OUTTAKECLOSE);

        followPath(path4);
//        linearSlideTargetHeight = -2350;
//        SLIDE_HEIGHT = -2350;
//        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftSlide.setPower(1);
//        robot.rightSlide.setPower(1);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
//        waitForLinearSlide(linearSlideTargetHeight);
        robot.smartServo.setPosition(Teleop.BASKET_SMARTSERVO);
        robot.arm.setPosition(Teleop.BASKET_ARMSERVO);
        sleep(1000);
        followPath(path2);
        sleep(500);
        robot.outtake.setPosition(0);

        followPath(path3);

//        linearSlideTargetHeight = 0;
//        SLIDE_HEIGHT = 0;
//        robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
//        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftSlide.setPower(0.5);
//        robot.rightSlide.setPower(0.5);
//        waitForLinearSlide(linearSlideTargetHeight);

        waitForStart();

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