/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;


/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Braking Calibration", group="Robot")

public class BrakingCalibration extends LinearOpMode {
    public IDRobot robot = new IDRobot();
    private double headingOffset = 0.0;
    private static final boolean FIELD_ORIENTED = false;

    private ArrayList<Pair<Double,Double>> brakingData = new ArrayList<Pair<Double, Double>>();
    @Override
    public void runOpMode() {
        robot.init(this);
        waitForStart();
        headingOffset = robot.odo.getHeading();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                robot.setPowers(1,1,1,1);
                while(gamepad1.dpad_up && opModeIsActive()) {
                    sleep(20);
                }
                robot.setPowers(0, 0, 0, 0);
                if (gamepad1.a) {
                    robot.setCoasting();
                    gatherBrakingData("double[][] forwardCoasting = { ");
                } else if (gamepad1.b) {
                    robot.setPowers(-0.1, -0.1, -0.1, -0.1);
                    robot.setBraking();
                    gatherBrakingData("double[][] forwardReversing = { ");
                } else {
                    robot.setPowers(0, 0, 0, 0);
                    robot.setBraking();
                    gatherBrakingData("double[][] forwardBraking = { ");
                }
                sleep(500);
            }
            if (gamepad1.dpad_down) {
                robot.setPowers(-1,-1,-1,-1);
                while(gamepad1.dpad_down) {
                    sleep(20);
                }
            }
            if (gamepad1.dpad_up  && gamepad1.left_bumper) {
                robot.setPowers(1,1,1,1);
                while(gamepad1.dpad_up && opModeIsActive()) {
                    sleep(20);
                }
                robot.setPowers(0,0,0,0);
                robot.setCoasting();
                gatherBrakingData("double[][] forwardCoasting = { ");
            }
            if (gamepad1.dpad_right && !gamepad1.left_bumper) {
                robot.setPowers(1,-1,1,-1);
                while(gamepad1.dpad_right && opModeIsActive()) {
                    sleep(20);
                }
                robot.setPowers(0,0,0,0);
                robot.setBraking();
                gatherBrakingData("double[][] strafeBraking = { ");
            }
            if (gamepad1.dpad_right  && gamepad1.left_bumper) {
                robot.setPowers(1,-1,1,-1);
                while(gamepad1.dpad_right && opModeIsActive()) {
                    sleep(20);
                }
                robot.setPowers(0,0,0,0);
                robot.setCoasting();
                gatherBrakingData("double[][] strafeCoasting = { ");
            }
            if (gamepad1.a) {
                robot.setPowers( -1, 1, 1, -1);
                while(gamepad1.a && opModeIsActive()) {
                    sleep(20);
                }
                robot.setPowers(0,0,0,0);
                robot.setCoasting();
                gatherTurningBrakingData("double[][] turnCoasting = { ");
            }
            if (gamepad1.b) {
                robot.setPowers( -1, 1, 1, -1);
                while(gamepad1.b && opModeIsActive()) {
                    sleep(20);
                }
                robot.setPowers(0,0,0,0);
                robot.setBraking();
                gatherTurningBrakingData("double[][] turnBraking = { ");
            }
            if (gamepad2.right_trigger > 0.3) {
                robot.disableLimits = true;
                if (gamepad2.left_trigger > 0.3) {
                    robot.armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.armRotationTarget = robot.armRotation.getCurrentPosition();
                }
            } else {
                robot.disableLimits = false;
            }


            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double x_transformed = x;
            double y_transformed = y;

            if (FIELD_ORIENTED) {
                double magnitude = Math.sqrt(y * y + x * x);
                double angle = Math.atan2(y, x);

                angle = angle - robot.odo.getPosition().getHeading(AngleUnit.RADIANS) - headingOffset;


                x_transformed = Math.cos(angle) * magnitude;
                y_transformed = Math.sin(angle) * magnitude;
            }

            if (gamepad1.a) {
                headingOffset = robot.odo.getHeading();
            }

            if (gamepad1.right_trigger > 0.3) {
                x_transformed = x_transformed * 0.3;
                y_transformed = y_transformed * 0.3;
                rx = rx * 0.4;
            }

            robot.leftFront.setPower(y_transformed + x_transformed * 1.1 + rx);
            robot.leftBack.setPower(y_transformed - x_transformed * 1.1 + rx);
            robot.rightFront.setPower(y_transformed - x_transformed * 1.1 - rx);
            robot.rightBack.setPower(y_transformed + x_transformed * 1.1 - rx);

            telemetry.addData("Heading", robot.odo.getPosition().getHeading(AngleUnit.RADIANS));

            //telemetry.addData("Heading", robot.odo.getHeading());
            telemetry.addData("X", robot.odo.getPosX());
            telemetry.addData("Y", robot.odo.getPosY());
            telemetry.addData("velocity", robot.getVelocity());

            telemetry.addData("lf", robot.leftFront.getPower());
            telemetry.addData("rf", robot.rightFront.getPower());
            telemetry.addData("lb", robot.leftBack.getPower());
            telemetry.addData("rb", robot.rightBack.getPower());
            telemetry.update();
            sleep(20);
            robot.odo.update();
        }
    }
    void gatherBrakingData(String output) {
        robot.odo.update();
        Pose2D startPosition = robot.odo.getPosition();
        double velocity = robot.getVelocity();
        double distance = 0.0;
        brakingData.clear();
        brakingData.add(new Pair<Double, Double>(distance,velocity));
        while ((velocity  > 1) && opModeIsActive()) {
            sleep(20);
            robot.odo.update();
            Pose2D currentPosition = robot.odo.getPosition();
            distance = robot.getDistance(startPosition,currentPosition);
            velocity = robot.getVelocity();
            brakingData.add(new Pair(distance, velocity));
        }
        double endDistance = brakingData.get(brakingData.size()-1).first;
        double brakingDistance;
        double distanceHere;
        for (int i = 0; i< brakingData.size() ; i++) {
            velocity = brakingData.get(i).second;
            distanceHere = brakingData.get(i).first;
            brakingDistance = endDistance - distanceHere;
            output = output.concat( "{" + velocity + " , " + brakingDistance + "}, " ) ;
            System.out.println("I: " + i + " Velocity: " + velocity + "Braking Distance: " + brakingDistance);
        }
        System.out.println(output  + " } ");

    }
    void gatherTurningBrakingData(String output) {
        robot.odo.update();
        Pose2D startPosition = robot.odo.getPosition();
        double velocity = robot.odo.getHeadingVelocity();
        double distance = 0.0;
        brakingData.clear();
        brakingData.add(new Pair<Double, Double>(distance,velocity));
        while ((velocity  > 1) && opModeIsActive()) {
            sleep(20);
            robot.odo.update();
            Pose2D currentPosition = robot.odo.getPosition();
            distance =  AngleUnit.normalizeDegrees(robot.getRotation(startPosition) - robot.getRotation(currentPosition));
            velocity = robot.odo.getHeadingVelocity();
            brakingData.add(new Pair(distance, velocity));
        }
        double endDistance = brakingData.get(brakingData.size()-1).first;
        double brakingDistance;
        double distanceHere;
        for (int i = 0; i< brakingData.size() ; i++) {
            velocity = brakingData.get(i).second;
            distanceHere = brakingData.get(i).first;
            brakingDistance = endDistance - distanceHere;
            output = output.concat( "{" + velocity + " , " + brakingDistance + "}, \n" ) ;
            System.out.println("I: " + i + " Velocity: " + velocity + "Braking Distance: " + brakingDistance);
        }
        System.out.println(output  + " } ");

    }
}

