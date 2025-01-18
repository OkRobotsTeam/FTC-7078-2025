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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


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

@TeleOp(name="Robot: TeleOp2", group="Robot")

public class IdTeleop2 extends LinearOpMode {
    public IDRobot robot = new IDRobot();
    private double headingOffset = 0.0;
    private static final boolean FIELD_ORIENTED = true;

    @Override
    public void runOpMode() {
        robot.init(this);
        waitForStart();
        headingOffset = robot.odo.getHeading();
        while (opModeIsActive()) {
            if (gamepad2.right_bumper) {
                robot.runIntakeIn();
                telemetry.addData("Intake", "Running Intake In");
            } else if (gamepad2.left_bumper) {
                robot.runIntakeOut();
            } else {
                robot.stopIntake();
            }
            if (gamepad2.dpad_up) {
                robot.setWristPosition(1.0);
                telemetry.addData("Wrist", "Setting Wrist Position to 1.0");
            }
            if (gamepad2.dpad_down) {
                robot.setWristPosition(0.0);
            }
            if (gamepad2.back) {
                if (robot.armState == IDRobot.ArmState.DOCKED) {
                    robot.startUndocking();
                }
            }
            if (gamepad2.start) {
                robot.armState = IDRobot.ArmState.DRIVING;
            }
            if (gamepad2.x) {
                robot.moveArmToDriving();
            }
            if (gamepad2.y) {
                robot.moveArmToScoring();
            }
            if (gamepad2.a) {
                robot.moveArmToPickup();
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
                telemetry.addData("Heading", robot.odo.getPosition().getHeading(AngleUnit.RADIANS));


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

//            double wristTrim = gamepad2.dpad_left ? 0.01: 0 + gamepad2.dpad_left ? -0.01: 0;

            robot.doArmControl(-gamepad2.left_stick_y, -gamepad2.right_stick_y, gamepad2.dpad_left, gamepad2.dpad_right);
            telemetry.addData("Rotation", robot.armRotation.getCurrentPosition());
            telemetry.addData("Extension", robot.armExtension.getCurrentPosition());
            telemetry.addData("Setting Wrist Position", robot.currentWristPosition);
            telemetry.addData("State", robot.armState.name());
            telemetry.addData("X", robot.odo.getPosX());
            telemetry.addData("Y", robot.odo.getPosY());
            telemetry.addData("rx", rx);
            telemetry.update();
            sleep(20);
            robot.odo.update();
        }
    }
}
