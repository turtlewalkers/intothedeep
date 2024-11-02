package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Config
@TeleOp
public class Teleop extends LinearOpMode {
    TurtleRobot robot = new TurtleRobot(this);
    double topPos, bottomPos;
    boolean softlock = true;
    public static double CONSTANT;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.topLeft.setPosition(1);
        robot.topRight.setPosition(1);
        robot.bottomRight.setPosition(0.41);
        robot.bottomLeft.setPosition(0.41);
        waitForStart();
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            robot.leftFront.setPower(y + x + rx / 2);
            robot.leftBack.setPower(y - x + rx / 2);
            robot.rightFront.setPower(y - x - rx / 2);
            robot.rightBack.setPower(y + x - rx / 2);

            if (gamepad2.a) {
                robot.leftHorizontalSlide.setPosition(1);
                robot.rightHorizontalSlide.setPosition(1);
                robot.topRight.setPosition(0.3);
                robot.topLeft.setPosition(0.3);
                if (!gamepad2.left_bumper) {
                    robot.intake.setPosition(1);
                }
                softlock = false;
            } else {
                robot.leftHorizontalSlide.setPosition(0);
                robot.rightHorizontalSlide.setPosition(0);
            }

            if (gamepad2.x) {
                robot.topRight.setPosition(0.3);
                robot.topLeft.setPosition(0.3);
                softlock = false;
            } else if (gamepad2.y) {
                robot.topRight.setPosition(1);
                robot.topLeft.setPosition(1);
                robot.bottomRight.setPosition(0.41);
                robot.bottomLeft.setPosition(0.41);
                softlock = true;
            }

            if (gamepad2.dpad_left) {
                robot.bottomRight.setPosition(0);
                robot.bottomLeft.setPosition(0);
            } else if (gamepad2.dpad_down && softlock) {
                robot.bottomRight.setPosition(1);
                robot.bottomLeft.setPosition(1);
            }

            if (gamepad2.right_bumper) {
                robot.intake.setPosition(1);
            } else if (gamepad2.left_bumper){
                robot.intake.setPosition(0.85);
            }
        }
    }
}
