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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;


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

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")

public class IdAuto extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftBack     = null;

    public DcMotor   rightBack   = null;

    private Pose2D targetStartPose;

    private Pose2D targetEndPose;

    private Pose2D currentPose;

    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.8;
    static final double     TURN_SPEED    = 0.5;

    enum MoveState {
        MOVING, BRAKING, STOPPED
    }
    private MoveState moveState = MoveState.STOPPED;

    private Pose2D startPosition;

    private double startRotation;

    private double moveDistance;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetStartPose = odo.getPosition();
        targetEndPose = odo.getPosition();
        currentPose = odo.getPosition();

        telemetry.addData(">", "Robot Ready.  Press START.");

        System.out.println("FIND ME!!!");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        // Step 1:  Drive forward for 3 seconds
//        leftFront.setPower(FORWARD_SPEED);
//        rightFront.setPower(FORWARD_SPEED);
//        rightBack.setPower(FORWARD_SPEED);
//        leftBack.setPower(FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (odo.getPosition().getX(DistanceUnit.CM) < 60)) {
//            odo.update();
//            Pose2D pos = odo.getPosition();
//            double centimetersLeft = 60 - odo.getPosition().getX(DistanceUnit.CM);
//            double powerCap = (centimetersLeft - 5) / 50;
//            double outputPower = Math.min(powerCap, FORWARD_SPEED);
//            leftFront.setPower(outputPower);
//            rightFront.setPower(outputPower);
//            rightBack.setPower(outputPower);
//            leftBack.setPower(outputPower);
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Position", data);      telemetry.addData("Position", data);
//            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//
//        }

        currentPose = new Pose2D(DistanceUnit.CM, currentPose.getX(DistanceUnit.CM), currentPose.getY(DistanceUnit.CM) - 60, AngleUnit.DEGREES, currentPose.getHeading(AngleUnit.DEGREES));
        move(60, 0.6);
        move(60,0.6);

        while (opModeIsActive() && moveState != MoveState.STOPPED) {
            strafeLoop();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);      telemetry.addData("Position", data);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        double loopEndPosition = (odo.getPosition().getHeading(AngleUnit.DEGREES));

        // Step 2:  Spin right for 1.3 seconds
//        leftFront.setPower(TURN_SPEED);
//        rightFront.setPower(TURN_SPEED);
//        rightBack.setPower(-TURN_SPEED);
//        leftBack.setPower(-TURN_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 3:  Drive Backward for 1 Second
//        leftFront.setPower(-FORWARD_SPEED);
//        rightFront.setPower(-FORWARD_SPEED);
//        rightBack.setPower(-FORWARD_SPEED);
//        leftBack.setPower(-FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }

        // Step 4:  Stop


        telemetry.addData("Path" , "Complete");
        telemetry.update();
        sleep(1000);

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
//        if (gamepad1.a) {
//            leftFront.setPower(1);
//        } else if (gamepad1.b){
//            leftFront.setPower(-1);
//        } else {
//            leftFront.setPower(0);
//        }

            if (gamepad1.right_bumper) {
                x = x * 0.2;
                y = y * 0.2;
                rx = rx * 0.2;
            }

            leftFront.setPower(y + x + rx);
            leftBack.setPower(y - x + rx);
            rightFront.setPower(y - x - rx);
            rightBack.setPower(y + x - rx);

            odo.update();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("End Loop Position", loopEndPosition);
            telemetry.update();
        }
    }

    private double turnDistance;

    private double moveSpeed;

    private double startBraking;

    private double desiredHeading;

    GoBildaPinpointDriver odo;

    private void setPowers(double lF, double rF, double rB, double lB) {
        leftFront.setPower(lF);
        rightFront.setPower(rF);
        rightBack.setPower(rB);
        leftBack.setPower(lB);
    }

    private void setPower(double power){
        setPowers(power, power, power, power);
    }

    public void move(double distance, double speed, double desiredHeading) {
        move(distance, speed);
        setDesiredHeading(desiredHeading);
    }
    public void setDesiredHeading(double desiredHeadingIn) {
        desiredHeading = desiredHeadingIn;
    }

    public double getCurrentRotation() {
        return -odo.getPosition().getHeading(AngleUnit.DEGREES);
    }

    private double getDistance(Pose2D a, Pose2D b) {
        double dx = a.getX(DistanceUnit.CM) - b.getX(DistanceUnit.CM);
        double dy = a.getY(DistanceUnit.CM) - b.getY(DistanceUnit.CM);
        return(Math.sqrt(dx * dx + dy * dy));
    }

    private double getVelocity() {
        double x = odo.getVelocity().getX(DistanceUnit.CM);
        double y = odo.getVelocity().getY(DistanceUnit.CM);
        return (Math.sqrt(x * x + y * y));
    }
    public void move(double distance, double speed) {

        moveState = MoveState.MOVING;
        startPosition = odo.getPosition();
        desiredHeading = targetStartPose.getHeading(AngleUnit.DEGREES);
        double targetX = targetStartPose.getX(DistanceUnit.CM) + (Math.cos(Math.toRadians(desiredHeading)) * distance);
        double targetY = targetStartPose.getY(DistanceUnit.CM) + (Math.sin(Math.toRadians(desiredHeading)) * distance);
        targetEndPose = new Pose2D(DistanceUnit.CM, targetX, targetY, AngleUnit.DEGREES, desiredHeading);
        moveDistance = Math.abs(distance);
        double xDifference = targetEndPose.getX(DistanceUnit.CM) - currentPose.getX(DistanceUnit.CM);
        double yDifference = targetEndPose.getY(DistanceUnit.CM) - currentPose.getY(DistanceUnit.CM);
        moveDistance = Math.hypot(xDifference, yDifference);
        desiredHeading = Math.toDegrees(Math.atan2(yDifference, xDifference));
        if (distance < 0) {
            moveSpeed = -speed;
        } else {
            moveSpeed = speed;
        }
        startBraking = 30;
        if (moveDistance < 60) {
            startBraking = moveDistance * 0.66;
        }
    }

    public void moveLoop() {
        odo.update();
        double distanceMoved = getDistance(startPosition, odo.getPosition());
        double distanceLeft = (moveDistance - distanceMoved);
        telemetry.addData("Distance Moved", distanceMoved);
        telemetry.addData("Distance Left", distanceLeft);
        System.out.println("Distance Moved: " + distanceMoved + " Distance Left: " + distanceLeft + " DX" + odo.getPosition().getX(DistanceUnit.CM));
        if (moveState == MoveState.MOVING) {
            double angularError = odo.getPosition().getHeading(AngleUnit.DEGREES) - desiredHeading;
            double adjust = angularError / 40;
            setPowers(moveSpeed + adjust, moveSpeed - adjust, moveSpeed - adjust, moveSpeed + adjust);
            System.out.println("Angle Error: " + angularError + " Adjust: " + adjust);
            if (distanceLeft < startBraking) {
                moveState = MoveState.BRAKING;
            }
        } else if (moveState == MoveState.BRAKING) {
            double velocity = getVelocity();
            double ratio = velocity / distanceLeft;
            if ((ratio > 8) || (distanceLeft < 0)) {
                System.out.println("BRAKING: Velocity: " + getVelocity() + " Distance Left : " + distanceLeft + " Ratio: " + ratio);

                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setPower(0);
            } else {
                System.out.println("COASTING: Velocity: " + getVelocity() + " Distance Left : " + distanceLeft + " Ratio: " + ratio);

                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                setPower(0);
            }
            if (getVelocity() < 1) {
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setPower(0);
                System.out.println("SETTING TO STOPPED");

                moveState = MoveState.STOPPED;
            }
        }
    }

    private double degreeDifference(double a, double b) {
        return AngleUnit.normalizeDegrees(a-b);
    }

    public void turn(double degrees, double speed) {
        moveState = MoveState.MOVING;
        startRotation = getCurrentRotation();
        turnDistance = Math.abs(degrees);
        moveSpeed = speed;
        if (degrees > 0) {
            System.out.println("Turning Left");
            setPowers(-moveSpeed, moveSpeed, moveSpeed, -moveSpeed);
        } else {
            System.out.println("Turning Right");
            setPowers(moveSpeed, -moveSpeed, -moveSpeed, moveSpeed);
        }
        startBraking = 25;
        if (turnDistance < 50) {
            startBraking = degrees * 0.66;
        }
    }


    public void turnLoop() {
                odo.update();
                double rotation = getCurrentRotation();
                double distanceTurned = degreeDifference(startRotation, rotation);
                double degreesLeft = turnDistance - Math.abs(distanceTurned);
                telemetry.addData("Distance Turned", distanceTurned);
                telemetry.addData("Degrees Left", degreesLeft);
                System.out.println("Distance Turned: " + distanceTurned + " Degrees Left: " + degreesLeft + " DX" + odo.getPosition().getX(DistanceUnit.CM));
                if (moveState == MoveState.MOVING) {
                    if (degreesLeft < startBraking) {
                        moveState = MoveState.BRAKING;
                    }
                } else if (moveState == MoveState.BRAKING) {
                    double velocity = getVelocity();
                    double ratio = velocity / degreesLeft;
                    if ((ratio > 8) || (degreesLeft < 0)) {
                        System.out.println("BRAKING: Velocity: " + getVelocity() + " Degrees Left : " + degreesLeft + " Ratio: " + ratio);

                        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        setPower(0);
                    } else {
                        System.out.println("COASTING: Velocity: " + getVelocity() + " Degrees Left : " + degreesLeft + " Ratio: " + ratio);

                        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        setPower(0);
                    }
                    if (getVelocity() < 1) {
                        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        setPower(0);
                        System.out.println("SETTING TO STOPPED");

                        moveState = MoveState.STOPPED;
                    }

                } else if (moveState == MoveState.STOPPED) {

                }
            }
    public void strafe(double distance, double speed) {
        moveState = MoveState.MOVING;
        startPosition = odo.getPosition();
        desiredHeading = startPosition.getHeading(AngleUnit.DEGREES);
        moveDistance = Math.abs(distance);
        if (distance < 0) {
            moveSpeed = -speed;
        } else {
            moveSpeed = speed;
        }
        startBraking = 20;
        if (moveDistance < 35) {
            startBraking = moveDistance * 0.66;
        }
    }

    public void strafeLoop() {
        odo.update();
        double distanceMoved = getDistance(startPosition, odo.getPosition());
        double distanceLeft = (moveDistance - distanceMoved);
        telemetry.addData("Distance Moved", distanceMoved);
        telemetry.addData("Distance Left", distanceLeft);
        System.out.println("Distance Moved: " + distanceMoved + " Distance Left: " + distanceLeft + " DX" + odo.getPosition().getX(DistanceUnit.CM));
        if (moveState == MoveState.MOVING) {
            double angularError = odo.getPosition().getHeading(AngleUnit.DEGREES) - desiredHeading;
            double adjust = angularError / 40;
            setPowers(-moveSpeed + adjust, moveSpeed - adjust, -moveSpeed - adjust, moveSpeed + adjust);
            System.out.println("Angle Error: " + angularError + " Adjust: " + adjust);
            if (distanceLeft < startBraking) {
                moveState = MoveState.BRAKING;
            }
        } else if (moveState == MoveState.BRAKING) {
            double velocity = getVelocity();
            double ratio = velocity / distanceLeft;
            if ((ratio > 9.5) || (distanceLeft < 0)) {
                System.out.println("BRAKING: Velocity: " + getVelocity() + " Distance Left : " + distanceLeft + " Ratio: " + ratio);

                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setPower(0);
            } else {
                System.out.println("COASTING: Velocity: " + getVelocity() + " Distance Left : " + distanceLeft + " Ratio: " + ratio);

                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                setPower(0);
            }
            if (getVelocity() < 1) {
                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setPower(0);
                System.out.println("SETTING TO STOPPED");

                moveState = MoveState.STOPPED;
            }
        }
    }


}
