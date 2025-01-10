package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;


public class IDRobot {

    public DcMotor leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftBack     = null;
    public DcMotor  rightBack   = null;

    public CRServo leftIntake = null;
    public CRServo rightIntake = null;
    public DcMotor armExtension = null;
    public DcMotor armRotation = null;
    public Servo wristRotation = null;
    GoBildaPinpointDriver odo;
    double currentWristPosition;
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

    private double getVelocity() {
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

        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setWristPosition(0.15);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    public void runIntakeIn () {
        leftIntake.setPower(1);
        rightIntake.setPower(-1);
}
    public void runIntakeOut () {
        leftIntake.setPower(-1);
        rightIntake.setPower(1);
    }
    public void stopIntake () {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }
    public void setWristPosition (double wristPosition) {
        wristPosition = Math.min(wristPosition, 0.7);
        wristPosition = Math.max(wristPosition, 0.15);
        currentWristPosition = wristPosition;
        wristRotation.setPosition(currentWristPosition);
        System.out.println("Setting Wrist Rotation: " + wristPosition);
    }
    public void extendArm (double power) {
        if (armExtension.getCurrentPosition() < 10) {
            power = Math.max(power, 0);
        } else if (armExtension.getCurrentPosition() > 7600) {
            power = Math.min(power, 0);
        }else if (armExtension.getCurrentPosition() < 150) {
            power = Math.max(power, -0.1);
        }
        armExtension.setPower(power);
    }

    public void extendArmToPosition (int position) {
        armExtension.setTargetPosition(position);
        armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtension.setPower(1);
    }
    public void rotateArmToPosition (int position) {
        armRotation.setTargetPosition(position);
        armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotation.setPower(1);
    }

    public void rotateArm (double power) {
        if (armRotation.getCurrentPosition() < -300) {
            power = Math.max(power, 0);
        } else if (armRotation.getCurrentPosition() > 6000) {
            power = Math.min(power, 0);
        }
        armRotation.setPower(power);
    }

    public void startUndocking () {
        armState = ArmState.UNDOCK;
        rotateArmToPosition(1600);
    }
    public void manualControl (double armExtensionTrim, double armRotateTrim, boolean wristTrimUp, boolean wristTrimDown) {
        if (wristTrimUp) {
            setWristPosition(currentWristPosition + 0.01);
        } else if (wristTrimDown) {
            setWristPosition(currentWristPosition - 0.01);
        }
        extendArm(armExtensionTrim);
        rotateArm(armRotateTrim);
    }

    public void moveArmToDriving () {
        if (armState == ArmState.PICKUP) {
            extendArmToPosition(10);
            armState = ArmState.PICKUP_TO_DRIVING_1;
        }else if (armState == ArmState.SCORING) {
            extendArmToPosition(10);
            armState = ArmState.SCORING_TO_DRIVING_1;
        }else if (armState == ArmState.DRIVING) {
            setWristPosition(0.15);
            extendArmToPosition(10);
            rotateArmToPosition(1600);
            armState = ArmState.SCORING_TO_DRIVING_1;
        }
    }
    public void moveArmToScoring () {
        if (armState == ArmState.PICKUP) {
            moveArmToDriving();
        }else if (armState == ArmState.DRIVING) {
            rotateArmToPosition(2200);
            armState = ArmState.DRIVING_TO_SCORING_1;
        }
    }
    public void moveArmToPickup () {
        if (armState == ArmState.SCORING) {
            moveArmToDriving();
        }else if (armState == ArmState.DRIVING) {
            rotateArmToPosition(0);
            setWristPosition(0.15);
            armState = ArmState.DRIVING_TO_PICKUP_1;
        }
    }
    public void doArmControl (double armExtensionTrim, double armRotateTrim, boolean wristTrimUp, boolean wristTrimDown) {
        if (armState == ArmState.DOCKED) {
            manualControl(armExtensionTrim, armRotateTrim, wristTrimUp, wristTrimDown);
        }else if (armState == ArmState.UNDOCK) {
            if (Math.abs(armRotation.getCurrentPosition() - 1600) < 100) {
                armState = ArmState.DRIVING;
                armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
            }
        }else if (armState == ArmState.DRIVING) {
            manualControl(armExtensionTrim, armRotateTrim, wristTrimUp, wristTrimDown);
        }else if (armState == ArmState.DRIVING_TO_PICKUP_1) {
            if (Math.abs(armRotation.getCurrentPosition() - 10) < 2) {
                extendArmToPosition(3000);
                armState = ArmState.DRIVING_TO_PICKUP_2;
            }
        }else if (armState == ArmState.DRIVING_TO_PICKUP_2) {
            if (Math.abs(armExtension.getCurrentPosition() - 2500) < 100) {
                armState = ArmState.PICKUP;
                armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
                armExtension.setPower(0);
            }
        }else if (armState == ArmState.PICKUP) {
            manualControl(armExtensionTrim, armRotateTrim, wristTrimUp, wristTrimDown);
        }else if (armState == ArmState.PICKUP_TO_DRIVING_1) {
            if (Math.abs(armExtension.getCurrentPosition() - 10) < 2000) {
                rotateArmToPosition(1600);
                armState = ArmState.PICKUP_TO_DRIVING_2;
            }
        }else if (armState == ArmState.PICKUP_TO_DRIVING_2) {
            if (Math.abs(armRotation.getCurrentPosition() - 1600) < 100) {
                armState = ArmState.DRIVING;
                armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
                armExtension.setPower(0);
            }
        }else if (armState == ArmState.DRIVING_TO_SCORING_1) {
            if (Math.abs(armRotation.getCurrentPosition() - 2200) < 3) {
                extendArmToPosition(7550);
                armState = ArmState.DRIVING_TO_SCORING_2;
            }
        }else if (armState == ArmState.DRIVING_TO_SCORING_2) {
            if (Math.abs(armExtension.getCurrentPosition() - 7550) < 100) {
                armState = ArmState.SCORING;
                armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
                armExtension.setPower(0);
            }
        }else if (armState == ArmState.SCORING) {
            manualControl(armExtensionTrim, armRotateTrim, wristTrimUp, wristTrimDown);
        }else if (armState == ArmState.SCORING_TO_DRIVING_1) {
            if (Math.abs(armExtension.getCurrentPosition() - 10) < 3000) {
                rotateArmToPosition(1600);
                armState = ArmState.PICKUP_TO_DRIVING_2;
            }
        }else if (armState == ArmState.SCORING_TO_DRIVING_2) {
            if (Math.abs(armRotation.getCurrentPosition() - 1600) < 100) {
                armState = ArmState.DRIVING;
                armRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armRotation.setPower(0);
                armExtension.setPower(0);
            }
        }else if (armState == ArmState.DRIVING_TO_DOCKED_1) {

        }
    }
}
