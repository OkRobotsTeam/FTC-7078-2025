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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;


@Autonomous(name="Robot: Id Observation Multiple Specimens", group="Robot")

public class IdObservationMultipleSpecimens extends LinearOpMode {
    IDRobot robot = new IDRobot();


    @Override
    public void runOpMode() {
        robot.init(this);
        waitForStart();

        robot.zeroPose();

        robot.move(10, 1, true);
        robot.runIntakeIn();
        sleep(20);
        robot.stopIntake();
//        robot.rotateArm(1);
//        while (robot.armRotation.getCurrentPosition() < 1400) {
//            sleep(20);
//        }
        robot.rotateArmToPosition(1700);
        robot.extendArmToPosition(-10);
        while (robot.armRotation.getCurrentPosition() < 400) {
            sleep(20);
        }
        robot.extendArmToPosition(2400);
        robot.setWristPosition(0.7);
        while (robot.armExtension.getCurrentPosition() < 2300 || robot.armRotation.getCurrentPosition() < 1600) {
            sleep(20);
        }

        robot.move(64, 1, true);
        robot.runIntakeIn();

        while (robot.armExtension.getCurrentPosition() < 1800) {
            sleep(20);
        }
        robot.extendArmToPosition(0);
        while (robot.armExtension.getCurrentPosition() > 630) {
            sleep(20);
        }

//        waitForA();
        robot.move(-30, 1, true);
//        waitForA();
        robot.stopIntake();

        robot.strafe(-70, 0.6, true);
//        waitForA();

        robot.move(80, 1, true);
//        waitForA();

        robot.strafe(-30, 0.6, false);
//        waitForA();

        robot.move(-92, 1, true);
//        waitForA();

        robot.move(95, 1, true);
//        waitForA();

        robot.strafe(-20, 0.6, false);
//        waitForA();

        robot.move(-93, 1, false);
//        waitForA();

        robot.move(20, 1, false);
//        waitForA();

        robot.turn(180, 0.6);
//        waitForA();
        robot.strafe(-8, 0.6, false);
//        robot.armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.armRotation.setPower(-1);
//        while (robot.armRotation.getCurrentPosition() > 400) {
//            sleep(20);
//        }
//        robot.armRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rotateArmToPosition(200);
        while (robot.armRotation.getCurrentPosition() > 500) {
            sleep(20);
        }
        robot.extendArmToPosition(1700);
        robot.setWristPosition(0.9);
        while (robot.armExtension.getCurrentPosition() < 400) {
            sleep(20);
        }
        robot.rotateArmToPosition(-250);
        while (robot.armRotation.getCurrentPosition() > -100) {
            sleep(20);
        }
        robot.runIntakeIn();
//        waitForA();
        robot.move(20, 0.3, false);
        sleep(300);
//        waitForA();
        robot.move(-15, 1, false);
//        robot.armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rotateArmToPosition(1700);
        robot.extendArmToPosition(300);
        while (robot.armRotation.getCurrentPosition() < 1500) {
            sleep(20);
        }
//        robot.armRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.stopIntake();
        robot.turn(180, 0.6);
//        waitForA();
        robot.strafe(110, 0.8, false);
        robot.extendArmToPosition(2400);
        while (robot.armExtension.getCurrentPosition() < 1800) {
            sleep(20);
        }
        robot.move(18, 1, false);
        robot.extendArmToPosition(0);
        robot.rotateArmToPosition(1300);
        while (robot.armExtension.getCurrentPosition() > 1800) {
            sleep(20);
        }
        robot.setWristPosition(0.0);
        robot.move(-20, 1, false);
        robot.rotateArmToPosition(100);



        double loopEndPosition = (robot.odo.getPosition().getHeading(AngleUnit.DEGREES));


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            if (gamepad1.right_bumper) {
                x = x * 0.2;
                y = y * 0.2;
                rx = rx * 0.2;
            }

            robot.leftFront.setPower(y + x + rx);
            robot.leftBack.setPower(y - x + rx);
            robot.rightFront.setPower(y - x - rx);
            robot.rightBack.setPower(y + x - rx);

            robot.odo.update();
            Pose2D pos = robot.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("End Loop Position", loopEndPosition);
            telemetry.addData("Path", "Complete");
            robot.displayTargetEndPose();
            telemetry.update();
        }
    }
    public void waitForA() {
        while (opModeIsActive() && gamepad1.a == false) {
            sleep(50);
        }
    }
}

