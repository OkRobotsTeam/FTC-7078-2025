package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;
import java.util.OptionalDouble;


public class IDRobot {
    static final double WRIST_MINIMUM_OUTPUT = 0.15;
    static final double WRIST_MAXIMUM_OUTPUT = 0.9;
    static final double ARM_EXTENSION_LIMIT = 2800;
    double currentWristPosition;

    public boolean disableLimits;

    public DcMotor leftFront, rightFront, leftBack, rightBack;
    public CRServo leftIntake, rightIntake;
    public DcMotorEx armExtension, armRotation;
    public Servo wristRotation;
    GoBildaPinpointDriver odo;
    private Pose2D targetStartPose;
    private Pose2D targetEndPose;
    private Pose2D currentPose;
    Telemetry telemetry;
    LinearOpMode opMode;
    BNO055IMU imu;
    enum MoveState {
        MOVING, BRAKING, STOPPED
    }
    private MoveState moveState = MoveState.STOPPED;

    private Pose2D startPosition;

    private double startRotation;

    private double moveDistance;

    public int armRotationTarget = 0;
    private boolean armRotationAuto = true;

    private ElapsedTime runtime = new ElapsedTime();
    public Pose2D zeroPose = new Pose2D(DistanceUnit.CM,0,0, AngleUnit.DEGREES, 0);


    public enum ArmState {
        DOCKED, SCORING,
        PICKUP,
        DRIVING,
        DRIVING_TO_PICKUP_1,
        DRIVING_TO_PICKUP_2,
        DRIVING_TO_SCORING_1,
        DRIVING_TO_SCORING_2,
        PICKUP_TO_DRIVING_1,
        PICKUP_TO_DRIVING_2,
        SCORING_TO_DRIVING_1,
        SCORING_TO_DRIVING_2,
        DRIVING_TO_DOCKED_1,
        SCORING_TO_PICKUP_1,
        PICKUP_TO_SCORING_1,
        UNDOCK
    }

    public IDRobot.ArmState armState = ArmState.DOCKED;

    private double getRobotVelocityCentimetersPerSecond() {
        double x = odo.getVelocity().getX(DistanceUnit.CM);
        double y = odo.getVelocity().getY(DistanceUnit.CM);
        return (Math.sqrt(x * x + y * y));
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init(LinearOpMode opModeIn) {
        // Define and Initialize Motors
        opMode = opModeIn;
        telemetry = opModeIn.telemetry;
        HardwareMap hardwareMap = opModeIn.hardwareMap;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        armExtension = hardwareMap.get(DcMotorEx.class, "armExtension");
        armRotation = hardwareMap.get(DcMotorEx.class, "armRotation");
        wristRotation = hardwareMap.get(Servo.class, "wristRotation");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        armExtension.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorConfigurationType fix = armRotation.getMotorType();
        fix.setMaxRPM(240);
        armRotation.setMotorType(fix);

        PIDFCoefficients coefficients = armRotation.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        System.out.println("PID Coefficients: " + coefficients.toString());
        coefficients.p = coefficients.p / 2;
        coefficients.f = 0;
        coefficients.d = 0;
        coefficients.i = 0;

        armRotation.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());


        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setOffsets(190, 0);
        zeroPose();
        odo.resetPosAndIMU();
        opMode.sleep(250);
        odo.update();
        targetStartPose = odo.getPosition();
        targetEndPose = odo.getPosition();
        currentPose = odo.getPosition();



    }

    public double clamp( double value, double lower_limit, double upper_limit) {
        value = Math.max(value, lower_limit);
        value = Math.min(value, upper_limit);
        return value;
    }
    public double clamp(double value, OptionalDouble lower_limit, OptionalDouble upper_limit) {
        if (lower_limit.isPresent()) {
            value = Math.max(value, lower_limit.getAsDouble());
        }
        if (upper_limit.isPresent()) {
            value = Math.min(value, upper_limit.getAsDouble());
        }
        return value;
    }

    public double linearInterpolate(double x, double y, double t) {
        if (x > y) throw new AssertionError("Y must be greater than X");
        return clamp((1 - t) * x + (t * y), OptionalDouble.of(x), OptionalDouble.of(y));
    }

    public void setWristPosition(double wristPosition) {
        wristPosition = linearInterpolate(WRIST_MINIMUM_OUTPUT, WRIST_MAXIMUM_OUTPUT, wristPosition);
        currentWristPosition = wristPosition;
        wristRotation.setPosition(currentWristPosition);
        System.out.println("Setting Wrist Rotation: " + wristPosition);
    }

    public void adjustWristPosition(double by) {
        currentWristPosition = currentWristPosition + by;
        currentWristPosition = Math.min(WRIST_MAXIMUM_OUTPUT,currentWristPosition);
        currentWristPosition = Math.max(WRIST_MINIMUM_OUTPUT,currentWristPosition);
        wristRotation.setPosition(currentWristPosition);
    }

    public boolean isWithin(int value1, int value2, int tolerance) {
//        if (Math.abs(value1 - value2) < tolerance){
//            System.out.println("IS_WITHIN: " + value1 + " : " + value2 + " : " + tolerance);
//        } else {
//            System.out.println("NOT_WITHIN: " + value1 + " : " + value2 + " : " + tolerance);
//        }
        return (Math.abs(value1 - value2) < tolerance);
    }
    public boolean armRotationIsWithin( int tolerance, int value) {
        return isWithin(armRotation.getCurrentPosition(), value, tolerance);
    }
    public boolean armExtensionIsWithin( int tolerance, int value) {
        return isWithin(armExtension.getCurrentPosition(), value, tolerance);
    }
    public void runIntakeIn() {
        leftIntake.setPower(-1);
        rightIntake.setPower(1);
    }

    public void runIntakeOut() {
        leftIntake.setPower(1);
        rightIntake.setPower(-1);
    }

    public void stopIntake() {
        //leftIntake.setPower(0);
        //rightIntake.setPower(0);
        leftIntake.getController().pwmDisable();
        rightIntake.getController().pwmDisable();
    }

    public void extendArm(double power) {
        if (disableLimits == false) {
            if (armExtension.getCurrentPosition() < 10) {
                power = Math.max(power, 0);
            } else if (armExtension.getCurrentPosition() > 4800) {
                power = Math.min(power, 0);
            } else if (armExtension.getCurrentPosition() < 150) {
                power = Math.max(power, -0.1);
            }
        }
        armExtension.setPower(power);
    }

    public void extendArmToPosition(int position) {
        armExtension.setTargetPosition(position);
        armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtension.setPower(0.7);
    }

    public void rotateArmToPosition(int position) {
        armRotation.setTargetPosition(position);
        armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotation.setPower(1);
    }
    private void rotateArmCustom(int position) {
//        armRotation.setTargetPosition(position);
//        armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armRotation.setPower(1);
        armRotationTarget = position;
        armRotationAuto = true;
    }

    public void rotateArmEnd() {
        //armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //armRotationAuto=false;
    }

    public void rotateArm(double power) {
        if (disableLimits == false) {
            if (armRotation.getCurrentPosition() < -300) {
                power = Math.max(power, 0);
            } else if (armRotation.getCurrentPosition() > 6000) {
                power = Math.min(power, 0);
            }
        }
        armRotation.setPower(power);
    }

    public void startUndocking() {
        armState = ArmState.UNDOCK;
        rotateArmCustom(1600);
        setWristPosition(0.9);
    }

    public void manualControl(double armExtensionTrim, double armRotateTrim, boolean wristTrimUp, boolean wristTrimDown) {
        if (wristTrimUp) {
            //setWristPosition(currentWristPosition + 0.01);
            adjustWristPosition(0.01);
        } else if (wristTrimDown) {
            adjustWristPosition(-0.01);

            //setWristPosition(currentWristPosition - 0.01);
        }

        boolean limitArmExtension = (armRotation.getCurrentPosition() < 2200) || armRotation.getCurrentPosition() > 3000;
        boolean limitArmRotation = (armRotation.getCurrentPosition() > 3000);

        if (limitArmExtension && (armExtension.getCurrentPosition() > ARM_EXTENSION_LIMIT)) {
            extendArm(-1);
        }else {
            extendArm(armExtensionTrim * 1);
        }
        if (limitArmRotation) {
            rotateArmCustom(2900);
        }else {
            if (Math.abs(armRotateTrim) > 0.2) {
                armRotationAuto = false;
                armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rotateArm(armRotateTrim * 0.7);
            } else {
                if (armRotationAuto == false) {
                    armRotationTarget = armRotation.getCurrentPosition();
                    armRotationAuto = true;
                }
            }
        }
    }

    public void moveArmToDriving() {
        if (armState == ArmState.PICKUP || armState == ArmState.SCORING) {
            extendArmToPosition(10);
            armState = ArmState.PICKUP_TO_DRIVING_1;
        } else if (armState == ArmState.DRIVING ||
                   armState == ArmState.SCORING_TO_DRIVING_1 ||
                   armState == ArmState.SCORING_TO_DRIVING_2 ) {
            setWristPosition(0.9);
            extendArmToPosition(10);
            armRotation.setPower(1);
            armState = ArmState.SCORING_TO_DRIVING_1;
        }
    }

    public void moveArmToScoring() {
        if (armState == ArmState.PICKUP) {
            armState = ArmState.PICKUP_TO_SCORING_1;
            rotateArmCustom(1900);
            extendArmToPosition(1700);
            setWristPosition(0.825);
        } else if (armState == ArmState.DRIVING) {
            rotateArmCustom(1900);
            setWristPosition(0.825);
            armState = ArmState.DRIVING_TO_SCORING_1;
        }
    }

    public void moveArmToPickup() {
        if (armState == ArmState.SCORING) {
            armState = ArmState.SCORING_TO_PICKUP_1;
            rotateArmCustom(-160);
            extendArmToPosition(1650);
            setWristPosition(1.0);
        } else if (armState == ArmState.DRIVING) {
            rotateArmCustom(-160);
            setWristPosition(1.0);
            armState = ArmState.DRIVING_TO_PICKUP_1;
        }
    }

    public void doArmControl(double armExtensionTrim, double armRotateTrim, boolean wristTrimUp, boolean wristTrimDown) {
        if (armState == ArmState.DOCKED) {
            manualControl(armExtensionTrim, armRotateTrim, wristTrimUp, wristTrimDown);
            telemetry.addData("Arm", "Doing Manual Control");
        } else if (armState == ArmState.UNDOCK) {
            if (armRotationIsWithin( 100, 1600)) {
                armState = ArmState.DRIVING;
                rotateArmEnd();
                armRotation.setPower(0);
            }
        } else if (armState == ArmState.DRIVING) {
            manualControl(armExtensionTrim, armRotateTrim, wristTrimUp, wristTrimDown);
            telemetry.addData("Arm", "Doing Manual Control");
        } else if (armState == ArmState.DRIVING_TO_PICKUP_1) {
            if (armRotationIsWithin(50, 160)) {
                extendArmToPosition(1650);
                armState = ArmState.DRIVING_TO_PICKUP_2;
            }
        } else if (armState == ArmState.DRIVING_TO_PICKUP_2) {
            if (armExtensionIsWithin(100, 1650)) {
                armState = ArmState.PICKUP;
                rotateArmEnd();
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0.3);
                armExtension.setPower(0);
            }
        } else if (armState == ArmState.PICKUP) {
            manualControl(armExtensionTrim, armRotateTrim, wristTrimUp, wristTrimDown);
            telemetry.addData("Arm", "Doing Manual Control");
        } else if (armState == ArmState.PICKUP_TO_DRIVING_1) {
            if (armExtensionIsWithin(100,10)) {
                rotateArmCustom(1600);
                armState = ArmState.PICKUP_TO_DRIVING_2;
            }
        } else if (armState == ArmState.PICKUP_TO_DRIVING_2) {
            if (armRotationIsWithin (100, 1600)) {
                armState = ArmState.DRIVING;
                rotateArmEnd();
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
                armExtension.setPower(0);
            }
        } else if (armState == ArmState.DRIVING_TO_SCORING_1) {
            if (armRotationIsWithin (100,1900)) {
                extendArmToPosition(1700);
                armState = ArmState.DRIVING_TO_SCORING_2;
            }
        } else if (armState == ArmState.DRIVING_TO_SCORING_2) {
            if (armExtensionIsWithin(100, 1700)) {
                armState = ArmState.SCORING;
                rotateArmEnd();
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
                armExtension.setPower(0);
            }
        } else if (armState == ArmState.SCORING) {
            manualControl(armExtensionTrim, armRotateTrim, wristTrimUp, wristTrimDown);
            telemetry.addData("Arm", "Doing Manual Control");
        } else if (armState == ArmState.SCORING_TO_DRIVING_1) {
            if (armExtensionIsWithin(2000,10)) {
                rotateArmCustom(1600);
                armState = ArmState.PICKUP_TO_DRIVING_2;
            }
        } else if (armState == ArmState.SCORING_TO_DRIVING_2) {
            if ((armRotationIsWithin (100,1600)) &&
                (armExtensionIsWithin(300, 10))) {
                armState = ArmState.DRIVING;
                armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
                armExtension.setPower(0);
            }
        } else if (armState == ArmState.DRIVING_TO_DOCKED_1) {

        } else if (armState == ArmState.PICKUP_TO_SCORING_1) {
            if (armRotationIsWithin(100, 1900)) {
                armState = ArmState.SCORING;
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
                armExtension.setPower(0);
            }
        } else if (armState == ArmState.SCORING_TO_PICKUP_1) {
            if (armRotationIsWithin(100, -160)) {
                armState = ArmState.PICKUP;
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
                armExtension.setPower(0);
            }
        }
        telemetry.update();
        //do PID control of armRotation motor here.
        if (armRotationAuto) {
            double diff = armRotation.getCurrentPosition() - armRotationTarget;
            double power = diff * -0.001;
            power = clamp(power, -1, 1);
            armRotation.setPower(power);
        }
    }



    public void zeroPose () {
        odo.setPosition(zeroPose);
    }
    private double turnDistance;

    private double moveSpeed;

    private double startBraking;

    private double desiredHeading;

    private void setPowers(double lF, double rF, double rB, double lB) {
        leftFront.setPower(lF);
        rightFront.setPower(rF);
        rightBack.setPower(rB);
        leftBack.setPower(lB);
    }

    private void setPower(double power){
        setPowers(power, power, power, power);
    }

    public void startMove(double distance, double speed, boolean correct, double desiredHeading) {
        startMove(distance, speed, correct);
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

    public void move(double distance, double speed, boolean correct) {
        startMove(distance, speed, correct);
        while (opMode.opModeIsActive() && moveState != MoveState.STOPPED) {
            moveLoop();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);      telemetry.addData("Position", data);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            displayTargetEndPose();
            telemetry.update();
        }
    }
    public void startMove(double distance, double speed, boolean correct) {
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
        if (correct) {
            moveDistance = Math.hypot(xDifference, yDifference);
            desiredHeading = Math.toDegrees(Math.atan2(yDifference, xDifference));
            System.out.println("Target X: " + targetX + " Target Y: " + targetY);
            System.out.println("Current X: " + currentPose.getX(DistanceUnit.CM) + " Current Y: " + currentPose.getY(DistanceUnit.CM));
            if (distance < 0) {
                desiredHeading = AngleUnit.normalizeDegrees(desiredHeading + 180);
                moveSpeed = -speed;
            } else {
                moveSpeed = speed;
            }
        } else {
            if (distance < 0) {
                moveSpeed = -speed;
            } else {
                moveSpeed = speed;
            }
        }

        System.out.println("Move Distance: " + distance + " | " + moveDistance + " Heading: " + targetStartPose.getHeading(AngleUnit.DEGREES) + " | " + desiredHeading);

        startBraking = 26;
        if (moveDistance < 40) {
            startBraking = moveDistance * 0.35 + 5;
        }
    }

    public void moveLoop() {
        odo.update();
        double distanceMoved = getDistance(startPosition, odo.getPosition());
        double distanceLeft = (moveDistance - distanceMoved);
        telemetry.addData("Distance Moved", distanceMoved);
        telemetry.addData("Distance Left", distanceLeft);
        if (moveState == MoveState.MOVING) {

            double angularError = odo.getPosition().getHeading(AngleUnit.DEGREES) - desiredHeading;
            angularError = AngleUnit.normalizeDegrees(angularError);
            double adjust = angularError / 40;
            setPowers(moveSpeed + adjust, moveSpeed - adjust, moveSpeed - adjust, moveSpeed + adjust);
            //System.out.println("Angle Error: " + angularError + " Adjust: " + adjust);
            System.out.println("MOVE - Distance Moved: " + distanceMoved + " Distance Left: " + distanceLeft + " Angle Error: " + angularError + " Adjust: " + adjust);

            if (distanceLeft < startBraking) {
                moveState = MoveState.BRAKING;
            }
        } else if (moveState == MoveState.BRAKING) {
            double velocity = getVelocity();
            double ratio = velocity / distanceLeft;
            if ((ratio > 8) || (distanceLeft < 0)) {
                System.out.println("MOVE BRAKING - Distance Moved: " + distanceMoved + " Distance Left: " + distanceLeft  + " Velocity: " + getVelocity()  + " Ratio: " + ratio );

                leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setPower(0);
            } else {
                System.out.println("MOVE COASTING - Distance Moved: " + distanceMoved + " Distance Left: " + distanceLeft  + " Velocity: " + getVelocity()  + " Ratio: " + ratio );

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
                System.out.println("X Error: " + (targetEndPose.getX(DistanceUnit.CM) - currentPose.getX(DistanceUnit.CM)) + " Y Error: " + (targetEndPose.getY(DistanceUnit.CM) - currentPose.getY(DistanceUnit.CM)));
                moveState = MoveState.STOPPED;
            }
        }
    }

    private double degreeDifference(double a, double b) {
        return AngleUnit.normalizeDegrees(a-b);
    }

    public void turn(double degrees, double speed) {
        startTurn(degrees, speed);
        while (opMode.opModeIsActive() && moveState != MoveState.STOPPED) {
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

    public void strafe(double distance, double speed, boolean correct) {
        startStrafe(distance, speed, correct);
        while (opMode.opModeIsActive() && moveState != MoveState.STOPPED) {
            strafeLoop();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);      telemetry.addData("Position", data);
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            displayTargetEndPose();
            telemetry.update();
        }
    }
    public void startStrafe(double distance, double speed, boolean correct) {
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
        if (correct) {
            moveDistance = Math.hypot(xDifference, yDifference);
            desiredHeading = Math.toDegrees(Math.atan2(yDifference, xDifference)) - 90;
            System.out.println("Target X: " + targetX + " Target Y: " + targetY);
            System.out.println("Current X: " + currentPose.getX(DistanceUnit.CM) + " Current Y: " + currentPose.getY(DistanceUnit.CM));
            if (distance < 0) {
                desiredHeading = AngleUnit.normalizeDegrees(desiredHeading + 180);
                moveSpeed = -speed;
            } else {
                moveSpeed = speed;
            }
        }  else {
            if (distance < 0) {
                moveSpeed = -speed;
            } else {
                moveSpeed = speed;
            }
        }
        System.out.println("Target X: " + targetX + " Target Y: " + targetY + "Start X: " + targetStartPose.getX(DistanceUnit.CM) + " Start Y: " + targetStartPose.getY(DistanceUnit.CM));
        System.out.println("Strafe Distance: " + distance + " | " + moveDistance + " Heading: " + targetStartPose.getHeading(AngleUnit.DEGREES) + " | " + desiredHeading);moveState = MoveState.MOVING;

        startBraking = 8;
        if (moveDistance < 35) {
            startBraking = moveDistance * 0.33;
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
            angularError = AngleUnit.normalizeDegrees(angularError);
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
                System.out.println("X Error: " + (targetEndPose.getX(DistanceUnit.CM) - currentPose.getX(DistanceUnit.CM)) + " Y Error: " + (targetEndPose.getY(DistanceUnit.CM) - currentPose.getY(DistanceUnit.CM)));
                moveState = MoveState.STOPPED;
            }
        }
    }


}
