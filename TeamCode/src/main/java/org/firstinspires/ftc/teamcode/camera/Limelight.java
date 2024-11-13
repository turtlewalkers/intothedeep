/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@TeleOp
public class Limelight extends LinearOpMode {

    private static final int FOCAL_LENGTH = 1320;
    private static final double WIDTH = 3.5;
    double left_command;
    double right_command;
    private Limelight3A limelight;
    TurtleRobot robot = new TurtleRobot(this);

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);

        robot.init(hardwareMap);
        robot.bottomLeft.setPosition(0.1);
        robot.bottomRight.setPosition(0.1);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

                    telemetry.addData("Botpose", botpose.toString());

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                        telemetry.addData("Area", cr.getTargetArea());

                        /** DISTANCE: **/
                        double y = 0;
                        for (List<Double> x: cr.getTargetCorners()) {
                            y += x.get(0);
                        }
                        y /= 2;
                        telemetry.addData("Width", y);
                        double distance = FOCAL_LENGTH * WIDTH / y;
                        telemetry.addData("Distance", distance);

                        /** CLAW ANGLE **/
                        telemetry.addData("corners", cr.getTargetCorners());
                        if (cr.getTargetCorners().size() == 4) {
                            List<List<Double>> corners = cr.getTargetCorners();
                            List<List<Double>> ogcorners = cr.getTargetCorners();
                            Collections.sort(corners, Comparator.comparingDouble(point -> point.get(1)));

                            List<Double> point1 = corners.get(0); // smallest y
                            List<Double> point2 = corners.get(1); // second smallest y

                            double theta;
                            if (point1.get(0) > point2.get(0)) {
                                theta = 1;
                            } else {
                                theta = -1;
                            }

                            // Calculate the angle in radians
                            theta *= Math.atan(Math.abs(point1.get(1) - point2.get(1)) / Math.abs(point1.get(0) - point2.get(0)));

                            // Using distance formula
                            double dist1 = Math.hypot(point1.get(0) - point2.get(0), point1.get(1) - point2.get(1));
                            int adjIdx = ogcorners.indexOf(point2);
                            ++adjIdx;
                            adjIdx %= 3;
                            List<Double> adjacentPoint = ogcorners.get(adjIdx);
                            double dist2 = Math.hypot(point2.get(0) - adjacentPoint.get(0), point2.get(1) - adjacentPoint.get(1));

                            telemetry.addData("dist1", dist1);
                            telemetry.addData("dist2", dist2);

                            if (Math.abs(dist1 - dist2) >= 50) {
                                theta += Math.PI / 2; // adding 90 degrees in radians
                            }

                            if (theta < 0) {
                                theta += Math.PI;
                            }

                            if (theta >= Math.PI / 2) {
                                theta -= Math.PI;
                            }

                            // Converting angle to degrees for better understanding
                            double angleInDegrees = Math.toDegrees(theta);
                            telemetry.addData("Angle of the sample", angleInDegrees);

                            robot.bottomRight.setPosition(0.8 + theta / Math.PI * 0.1);
                            robot.bottomLeft.setPosition(0.8 - theta / Math.PI * 0.1);
                        }
                        telemetry.update();

                        /** ALIGN TO ANGLE **/
                        double Kp = -0.1f;
                        double min_command = 0.05f;

                        if (gamepad1.a)
                        {
                            double heading_error = result.getTx();
                            double steering_adjust = 0.0f;
                            if (Math.abs(heading_error) > 1.0)
                            {
                                if (heading_error < 0)
                                {
                                    steering_adjust = Kp*heading_error + min_command;
                                }
                                else
                                {
                                    steering_adjust = Kp*heading_error - min_command;
                                }
                            }
                            left_command += steering_adjust;
                            right_command -= steering_adjust;
                            robot.leftFront.setPower(left_command);
                            robot.leftBack.setPower(left_command);
                            robot.rightFront.setPower(right_command);
                            robot.rightBack.setPower(right_command);
                        }
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
                telemetry.update();
            }


        }
        limelight.stop();
    }
}