package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Config
@TeleOp
public class PIDF extends OpMode {
    TurtleRobot robot = new TurtleRobot(this);
    private PIDController controller;
    public static double p = 0.02, i = 0, d = 0.0001;
    public static double f = 0.2;
    public static int target = 0;
    public static final double ticks_in_degrees = 537.7;
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int linearSlidePosition = robot.leftSlide.getCurrentPosition();
        double pid = controller.calculate(linearSlidePosition, target);
        double ff = Math.toRadians(target / ticks_in_degrees) * f;
        double power = pid + ff;
        robot.leftSlide.setPower(power);
        robot.rightSlide.setPower(power);

        telemetry.addData("pos", linearSlidePosition);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
