package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Config
@Autonomous
public class RedLeft extends LinearOpMode {
    TurtleRobot robot = new TurtleRobot(this);
    @Override
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        robot.topLeft.setPosition(1);
        robot.topRight.setPosition(1);
        robot.leftHorizontalSlide.setPosition(0);
        robot.rightHorizontalSlide.setPosition(0);
        waitForStart();
        if (opModeIsActive()) {
            robot.rightFront.setPower(0.5);
            robot.leftFront.setPower(0.5);
            robot.leftBack.setPower(0.5);
            robot.rightBack.setPower(0.5);
            sleep(600);
            robot.rightFront.setPower(0);
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            robot.topLeft.setPosition(1);
            robot.topRight.setPosition(1);
            robot.leftHorizontalSlide.setPosition(0);
            robot.rightHorizontalSlide.setPosition(0);
            sleep(2000);
            robot.topLeft.setPosition(1);
            robot.topRight.setPosition(1);
            robot.leftHorizontalSlide.setPosition(0);
            robot.rightHorizontalSlide.setPosition(0);
        }
    }
}
