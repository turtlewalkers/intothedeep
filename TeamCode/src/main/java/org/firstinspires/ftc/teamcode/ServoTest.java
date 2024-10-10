package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class ServoTest extends LinearOpMode {
    CRServo right;
    CRServo left;
    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.get(CRServo.class, "right");
        left = hardwareMap.get(CRServo.class, "left");
        left.setPower(0);
        right.setPower(0);
        waitForStart();

        while (opModeIsActive()) {
            left.setPower(1);
            right.setPower(-1);
        }
    }
}
