package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.OptionalDouble;


public class IDRobot {
    static final double WRIST_MINIMUM_OUTPUT = 0.15;
    static final double WRIST_MAXIMUM_OUTPUT = 0.9;
    static final double ARM_EXTENSION_LIMIT = 4500;
    double currentWristPosition;

    public boolean disableLimits;

    public DcMotor leftFront, rightFront, leftBack, rightBack;
    public CRServo leftIntake, rightIntake;
    public DcMotor armExtension, armRotation;
    public Servo wristRotation;
    GoBildaPinpointDriver odo;
    BNO055IMU imu;

    public Pose2D zeroPose = new Pose2D(DistanceUnit.CM,0,0, AngleUnit.DEGREES, 0);


    public enum ArmState {
        DOCKED,
        SCORING,
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
    public void init(HardwareMap hardwareMap) {
        // Define and Initialize Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        armExtension = hardwareMap.get(DcMotor.class, "armExtension");
        armRotation = hardwareMap.get(DcMotor.class, "armRotation");
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

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(new BNO055IMU.Parameters());

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();


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

    public void runIntakeIn() {
        leftIntake.setPower(1);
        rightIntake.setPower(-1);
    }

    public void runIntakeOut() {
        leftIntake.setPower(-1);
        rightIntake.setPower(1);
    }

    public void stopIntake() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    public void extendArm(double power) {
        if (disableLimits == false) {
            if (armExtension.getCurrentPosition() < 10) {
                power = Math.max(power, 0);
            } else if (armExtension.getCurrentPosition() > 7600) {
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
        armExtension.setPower(1);
    }

    public void rotateArmToPosition(int position) {
        armRotation.setTargetPosition(position);
        armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotation.setPower(1);
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
        rotateArmToPosition(1600);
    }

    public void manualControl(double armExtensionTrim, double armRotateTrim, boolean wristTrimUp, boolean wristTrimDown) {
        if (wristTrimUp) {
            setWristPosition(currentWristPosition + 0.01);
        } else if (wristTrimDown) {
            setWristPosition(currentWristPosition - 0.01);
        }

        boolean limitArmExtension = (armRotation.getCurrentPosition() < 2200) || armRotation.getCurrentPosition() > 3000;

        if (limitArmExtension && (armExtension.getCurrentPosition() > ARM_EXTENSION_LIMIT)) {
            extendArm(-1);
        }else {
            extendArm(armExtensionTrim);
        }
        rotateArm(armRotateTrim);
    }

    public void moveArmToDriving() {
        if (armState == ArmState.PICKUP || armState == ArmState.SCORING) {
            extendArmToPosition(10);
            armState = ArmState.PICKUP_TO_DRIVING_1;
        } else if (armState == ArmState.DRIVING ||
                   armState == ArmState.SCORING_TO_DRIVING_1 ||
                   armState == ArmState.SCORING_TO_DRIVING_2 ) {
            setWristPosition(0.0);
            extendArmToPosition(10);
            rotateArmToPosition(1600);
            armState = ArmState.SCORING_TO_DRIVING_1;
        }
    }

    public void moveArmToScoring() {
        if (armState == ArmState.PICKUP) {
            moveArmToDriving();
        } else if (armState == ArmState.DRIVING) {
            rotateArmToPosition(2700);
            armState = ArmState.DRIVING_TO_SCORING_1;
        }
    }

    public void moveArmToPickup() {
        if (armState == ArmState.SCORING) {
            moveArmToDriving();
        } else if (armState == ArmState.DRIVING) {
            rotateArmToPosition(400);
            extendArmToPosition(1000);
            setWristPosition(1.0);
            armState = ArmState.DRIVING_TO_PICKUP_1;
        }
    }

    public void doArmControl(double armExtensionTrim, double armRotateTrim, boolean wristTrimUp, boolean wristTrimDown) {
        if (armState == ArmState.DOCKED) {
            manualControl(armExtensionTrim, armRotateTrim, wristTrimUp, wristTrimDown);
        } else if (armState == ArmState.UNDOCK) {
            if (Math.abs(armRotation.getCurrentPosition() - 1600) < 100) {
                setWristPosition(0.0);
                armState = ArmState.DRIVING;
                armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
            }
        } else if (armState == ArmState.DRIVING) {
            manualControl(armExtensionTrim, armRotateTrim, wristTrimUp, wristTrimDown);
        } else if (armState == ArmState.DRIVING_TO_PICKUP_1) {
            if ((Math.abs(armRotation.getCurrentPosition() - 400) < 50) &&
                (armExtension.getCurrentPosition() > 700)) {
                extendArmToPosition(3000);
                rotateArmToPosition(100);
                armState = ArmState.DRIVING_TO_PICKUP_2;
            }
        } else if (armState == ArmState.DRIVING_TO_PICKUP_2) {
            if (Math.abs(armExtension.getCurrentPosition() - 2500) < 100) {
                armState = ArmState.PICKUP;
                armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0.3);
                armExtension.setPower(0);
            }
        } else if (armState == ArmState.PICKUP) {
            manualControl(armExtensionTrim, armRotateTrim, wristTrimUp, wristTrimDown);
        } else if (armState == ArmState.PICKUP_TO_DRIVING_1) {
            if (Math.abs(armExtension.getCurrentPosition() - 10) < 2000) {
                rotateArmToPosition(1600);
                armState = ArmState.PICKUP_TO_DRIVING_2;
            }
        } else if (armState == ArmState.PICKUP_TO_DRIVING_2) {
            if (Math.abs(armRotation.getCurrentPosition() - 1600) < 100) {
                armState = ArmState.DRIVING;
                armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
                armExtension.setPower(0);
            }
        } else if (armState == ArmState.DRIVING_TO_SCORING_1) {
            if (Math.abs(armRotation.getCurrentPosition() - 2200) < 100) {
                extendArmToPosition(7650);
                armState = ArmState.DRIVING_TO_SCORING_2;
            }
        } else if (armState == ArmState.DRIVING_TO_SCORING_2) {
            if (Math.abs(armExtension.getCurrentPosition() - 7550) < 100) {
                armState = ArmState.SCORING;
                armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
                armExtension.setPower(0);
            }
        } else if (armState == ArmState.SCORING) {
            manualControl(armExtensionTrim, armRotateTrim, wristTrimUp, wristTrimDown);
        } else if (armState == ArmState.SCORING_TO_DRIVING_1) {
            if (Math.abs(armExtension.getCurrentPosition() - 10) < 2000) {
                rotateArmToPosition(1600);
                armState = ArmState.PICKUP_TO_DRIVING_2;
            }
        } else if (armState == ArmState.SCORING_TO_DRIVING_2) {
            if ((Math.abs(armRotation.getCurrentPosition() - 1600) < 100) &&
                (Math.abs(armExtension.getCurrentPosition() - 10) < 300)) {
                armState = ArmState.DRIVING;
                armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
                armExtension.setPower(0);
            }
        } else if (armState == ArmState.DRIVING_TO_DOCKED_1) {

        }
    }
}
