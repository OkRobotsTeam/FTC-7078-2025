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

@Autonomous(name="Robot: Id Net Specimen", group="Robot")

public class IdNetSpecimen extends LinearOpMode {

    /* Declare OpMode members. */
    public IDRobot robot = new IDRobot();
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
        robot.init(hardwareMap);

        leftFront = robot.leftFront;
        rightFront = robot.rightFront;
        rightBack = robot.rightBack;
        leftBack = robot.leftBack;

        telemetry.addData(">", "Robot Ready.  Press START.");

        odo = robot.odo;
        telemetry.update();

        targetStartPose = odo.getPosition();
        targetEndPose = odo.getPosition();
        currentPose = odo.getPosition();
        sleep(300);
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

        robot.odo.setPosition(robot.zeroPose);
        move(10,0.6);
        robot.rotateArmToPosition(1700);
        robot.extendArmToPosition(-10);
        while (robot.armRotation.getCurrentPosition() < 400) {
            sleep(20);
        }
        robot.extendArmToPosition(3800);
        robot.setWristPosition(0.7);
        while (robot.armExtension.getCurrentPosition() < 3700 || robot.armRotation.getCurrentPosition() <1600) {
            sleep(20);
        }
        move(64, 0.4);
        while (robot.armExtension.getCurrentPosition() < 2900) {
            sleep(20);
        }
        robot.extendArmToPosition(0);
        while (robot.armExtension.getCurrentPosition() > 1000) {
            sleep(20);
        }
        move(-30, 0.6);
        robot.rotateArmToPosition(0);
        robot.setWristPosition(0.0);
        strafe(120, 0.6);

//        move(60, 0.6);
//        waitForA();
//        turn(90, 0.6);
//        waitForA();
//        move(60,0.6);
//        waitForA();
//        turn(90,0.6);
//        waitForA();
//        strafe(60,0.6);
//        waitForA();
//        strafe(-60, 0.6);
//        waitForA();
//        move(60, 0.6);
//        waitForA();
//        turn(-45,0.6);
//        waitForA();
//        move(-60 * 1.414, 0.6);
//        waitForA();
//        turn(-135,0.6);
//        waitForA();
//        move(-60, 0.6);





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
            telemetry.addData("Path" , "Complete");
            displayTargetEndPose();
            telemetry.update();
        }
    }

    public void waitForA() {
        while (opModeIsActive() && gamepad1.a == false) {
            sleep(50);
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

    public void startMove(double distance, double speed, double desiredHeading) {
        startMove(distance, speed);
        setDesiredHeading(desiredHeading);
    }
    public void setDesiredHeading(double desiredHeadingIn) {
        desiredHeading = desiredHeadingIn;
    }

    public double getRotation(Pose2D pose) {
        return -pose.getHeading(AngleUnit.DEGREES);
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

    public void displayTargetEndPose () {
        telemetry.addData("Target End Pose X", targetEndPose.getX(DistanceUnit.CM));
        telemetry.addData("Target End Pose Y", targetEndPose.getY(DistanceUnit.CM));
        telemetry.addData("Target End Pose Heading", targetEndPose.getHeading(AngleUnit.DEGREES));
    }

    public void move(double distance, double speed) {
        startMove(distance, speed);
        while (opModeIsActive() && moveState != MoveState.STOPPED) {
            moveLoop();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);      telemetry.addData("Position", data);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            displayTargetEndPose();
            telemetry.update();
        }
    }
    public void startMove(double distance, double speed) {
        moveState = MoveState.MOVING;
        targetStartPose = targetEndPose;
        currentPose = odo.getPosition();
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
            desiredHeading = AngleUnit.normalizeDegrees(desiredHeading+180);
            moveSpeed = -speed;
        } else {
            moveSpeed = speed;
        }

        System.out.println("Move Distance: " + distance + " | " + moveDistance + " Heading: " + targetStartPose.getHeading(AngleUnit.DEGREES) + " | " + desiredHeading);

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
        startTurn(degrees, speed);
        while (opModeIsActive() && moveState != MoveState.STOPPED) {
            turnLoop();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);      telemetry.addData("Position", data);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            displayTargetEndPose();
            telemetry.update();
        }
    }
    public void startTurn(double degrees, double speed) {
        moveState = MoveState.MOVING;
        targetStartPose = targetEndPose;
        currentPose = odo.getPosition();
        startPosition = odo.getPosition();
        startRotation = startPosition.getHeading(AngleUnit.DEGREES);
        desiredHeading = targetStartPose.getHeading(AngleUnit.DEGREES) + degrees;
        targetEndPose = new Pose2D(DistanceUnit.CM, targetStartPose.getX(DistanceUnit.CM), targetStartPose.getY(DistanceUnit.CM), AngleUnit.DEGREES, desiredHeading);
        double turnDegrees = desiredHeading - startRotation;
        turnDistance = Math.abs(turnDegrees);
        moveSpeed = speed;
        System.out.println("Desired Heading: " + desiredHeading + " Target Start Degrees: " + targetStartPose.getHeading(AngleUnit.DEGREES) + " Current Degrees: " + startRotation);
        if (turnDegrees > 0) {
            System.out.println("Turning Left " + turnDegrees);
            setPowers(-moveSpeed, moveSpeed, moveSpeed, -moveSpeed);
        } else {
            System.out.println("Turning Right " + turnDegrees);
            setPowers(moveSpeed, -moveSpeed, -moveSpeed, moveSpeed);
        }
        startBraking = 25;
        if (turnDistance < 50) {
            startBraking = turnDistance * 0.66;
        }
    }


    public void turnLoop() {
                odo.update();
                double rotation = odo.getPosition().getHeading(AngleUnit.DEGREES);
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
        startStrafe(distance, speed);
        while (opModeIsActive() && moveState != MoveState.STOPPED) {
            strafeLoop();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);      telemetry.addData("Position", data);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            displayTargetEndPose();
            telemetry.update();
        }
    }
    public void startStrafe(double distance, double speed) {
        moveState = MoveState.MOVING;
        targetStartPose = targetEndPose;
        currentPose = odo.getPosition();
        startPosition = odo.getPosition();
        double startHeading = targetStartPose.getHeading(AngleUnit.DEGREES);
        double moveDirection = startHeading + 90;
        double targetX = targetStartPose.getX(DistanceUnit.CM) + (Math.cos(Math.toRadians(moveDirection)) * distance);
        double targetY = targetStartPose.getY(DistanceUnit.CM) + (Math.sin(Math.toRadians(moveDirection)) * distance);
        targetEndPose = new Pose2D(DistanceUnit.CM, targetX, targetY, AngleUnit.DEGREES, startHeading);
        moveDistance = Math.abs(distance);
        double xDifference = targetEndPose.getX(DistanceUnit.CM) - currentPose.getX(DistanceUnit.CM);
        double yDifference = targetEndPose.getY(DistanceUnit.CM) - currentPose.getY(DistanceUnit.CM);
        moveDistance = Math.hypot(xDifference, yDifference);
        desiredHeading = Math.toDegrees(Math.atan2(yDifference, xDifference)) - 90;
        if (distance < 0) {
            desiredHeading = AngleUnit.normalizeDegrees(desiredHeading+180);
            moveSpeed = -speed;
        } else {
            moveSpeed = speed;
        }
        System.out.println("Target X: " + targetX + " Target Y: " + targetY + "Start X: " + targetStartPose.getX(DistanceUnit.CM) + " Start Y: " + targetStartPose.getY(DistanceUnit.CM));
        System.out.println("Strafe Distance: " + distance + " | " + moveDistance + " Heading: " + targetStartPose.getHeading(AngleUnit.DEGREES) + " | " + desiredHeading);moveState = MoveState.MOVING;

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
