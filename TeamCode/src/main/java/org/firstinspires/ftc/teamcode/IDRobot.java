package org.firstinspires.ftc.teamcode;
import android.view.MotionEvent;

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
    static double[][] forwardBraking = {
            {145.53928321348127 , 28.192262887836076},
            {126.31458524003165 , 24.688209240512272},
            {121.3517454150427 , 21.349858743639047},
            {119.16693445185768 , 18.141148527689765},
            {100.50022941658122 , 15.269055214146393},
            {84.318875940517 , 12.905176545547171},
            {72.92509262430072 , 10.76657545079268},
            {65.40469641177792 , 8.783315674862035},
            {55.06680311870858 , 7.1786875010745135},
            {49.41664608664455 , 5.864861715414513},
            {39.988745247058354 , 4.653367578128723},
            {33.65885766922857 , 3.600330453878737},
            {29.007258547003932 , 2.730862453760224},
            {25.347470042623538 , 2.012657576787177},
            {17.549890416294744 , 1.4484510336029253},
            {16.616289611705973 , 1.010311678781548},
            {11.863239198786456 , 0.6290732006413116},
            {8.86450517892052 , 0.3642940017588785},
            {4.52272743664058 , 0.17394542970094662},
            {2.4213975956110736 , 0.053284790988669783},
            {2.5340008289254663 , 0.002744849185599918},
            {0.005538939912091901 , 0.0}  };
    static double[][] forwardCoasting = { {131.8715955065602 , 89.07231696922665},
            {127.51689080145833 , 85.75704776087511},
            {129.3296604719609 , 82.20759361877265},
            {129.74517771547826 , 78.53832799099544},
            {129.64100692065406 , 75.19275585988987},
            {123.91580566390341 , 71.84303858025477},
            {125.26050603790027 , 68.56112447679692},
            {115.33361223237851 , 65.2337297288539},
            {114.26076838270167 , 62.01178517307381},
            {108.78125737069189 , 59.00824393304664},
            {106.21940317279848 , 56.062686594503475},
            {111.65031514714698 , 53.194076165869994},
            {102.31155101782107 , 50.25089647859814},
            {100.86297075179452 , 47.59201424808277},
            {92.79390643727402 , 44.99752029860899},
            {95.92162079730599 , 42.467333628323026},
            {89.09819727722761 , 40.09056230554766},
            {87.51852319291557 , 37.71410559476692},
            {89.02355508244165 , 35.43838944853316},
            {84.48623804042191 , 33.143988069338555},
            {80.70933616745926 , 31.004222273962895},
            {76.29689242605046 , 28.984223108294216},
            {74.76390209137114 , 27.033298130409214},
            {75.45751608698215 , 25.19031570849431},
            {68.44156603506066 , 23.429867130593024},
            {64.95576572003567 , 21.689647512658098},
            {61.4708277635224 , 20.047963618895736},
            {60.10019960113185 , 18.414060988380257},
            {58.927899269861406 , 16.9490046684368},
            {52.87199413767831 , 15.52770720372527},
            {50.90603144657989 , 14.160204090800107},
            {50.28628912629398 , 12.800279408909319},
            {47.24557592640315 , 11.515015906204454},
            {44.40250093114885 , 10.337570524080135},
            {42.17896735222706 , 9.22202217105675},
            {40.730095052507465 , 8.149001100488647},
            {36.437773605431744 , 7.133554579158854},
            {32.35522575172326 , 6.213201605315035},
            {30.180676536248743 , 5.397279924178008},
            {30.045769260814428 , 4.632598859586096},
            {24.833898897769554 , 3.867504592819273},
            {23.031926010336022 , 3.2041136719600587},
            {23.352685455801655 , 2.6035320175694494},
            {18.722730785928576 , 2.0917076841604683},
            {16.235944934863 , 1.6483916412531983},
            {13.597939275289107 , 1.2303227043859266},
            {11.318624096628762 , 0.8769947270420033},
            {9.967343355132064 , 0.5843158335942036},
            {7.920874006225316 , 0.3502320744006937},
            {5.611730843364376 , 0.1867908922227599},
            {3.070299672957141 , 0.05524932307838526},
            {0.8228271381112563 , 0.0}  };
    static  double[][] strafeBraking = { {76.71871053429271 , 7.907685797647719},
            {58.81316267575046 , 6.036483707861949},
            {53.287557448209554 , 4.588484213585077},
            {49.03960369490483 , 3.264514836601065},
            {42.44094689595753 , 2.0229173005369887},
            {27.548336078151614 , 1.0764860557894513},
            {16.413149029269526 , 0.5125186173781247},
            {12.698472495430535 , 0.17514024733997502},
            {0.012476490180030317 , 0.0}  };
    double[][] strafeCoasting = { {105.32452213971925 , 24.8485338159127},
            {93.42290687340146 , 22.325194198676506},
            {91.42961081410286 , 19.932651615923287},
            {92.48288887579034 , 17.565209555998038},
            {89.16092024556947 , 15.338600683085986},
            {76.51752915726482 , 13.317968067504385},
            {67.68401296670824 , 11.351242880159797},
            {60.83725776033927 , 9.61326504091607},
            {60.29470970894108 , 8.012966821340708},
            {52.544102454694325 , 6.482168728631894},
            {47.931742142678964 , 5.027391035516281},
            {43.40105738046177 , 3.8570138966450287},
            {35.48573963803778 , 2.7952789814676677},
            {29.525677192114696 , 1.9808075641183116},
            {25.517794545046023 , 1.2876726852016631},
            {19.25265138200989 , 0.7447000113270512},
            {12.834922668035489 , 0.37598620099522506},
            {7.7437858139917175 , 0.12990117033334414},
            {0.18058294048086257 , 0.0}  };
    double[][] turnCoasting = {
            {4.5714592933654785 , 46.02535266553599},
            {3.9597175121307373 , 39.788078556993696},
            {3.733032464981079 , 33.82155314715072},
            {3.292947769165039 , 28.560900467271892},
            {3.0264408588409424 , 23.76519242457414},
            {2.8081743717193604 , 19.201246360793277},
            {2.5235493183135986 , 15.140480333167261},
            {2.1830427646636963 , 11.614900608146797},
            {1.914730429649353 , 8.360479393175808},
            {1.6263841390609741 , 5.767056565321923},
            {1.4072757959365845 , 3.50361383664972},
            {1.1409660577774048 , 1.4664142940450233},
            {0.8291528820991516 , 0.0}};
    double[][] turnBraking = {
            {4.863333225250244 , 18.357881922836782},
            {3.814893960952759 , 12.072304716484155},
            {2.8949763774871826 , 7.225001424796119},
            {1.8881874084472656 , 3.6205193544836334},
            {1.1868184804916382 , 1.325849000926155},
            {0.6549578309059143 , 0.0}};

    double currentWristPosition;

    public boolean disableLimits;

    public DcMotor leftFront, rightFront, leftBack, rightBack;
    public CRServo leftIntake, rightIntake;
    public DcMotorEx armExtension, armRotation;
    public Servo wristRotation;
    GoBildaPinpointDriver odo;
    //FakeOdo odo = new FakeOdo();
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

        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFCoefficients coefficients = armRotation.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        System.out.println("PID Coefficients: " + coefficients.toString());
        coefficients.p = coefficients.p / 2;
        coefficients.f = 0;
        coefficients.d = 0;
        coefficients.i = 0;

        armRotation.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);


        //odo.init(this);  //if FakeOdo
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
        //telemetry.update();
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

    private double movePower;

    private double startBraking;

    private double desiredHeading;

    public void setPowers(double lF, double rF, double rB, double lB) {
        leftFront.setPower(lF);
        rightFront.setPower(rF);
        rightBack.setPower(rB);
        leftBack.setPower(lB);
    }

    public void setBraking() {
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setCoasting() {
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setPower(double power){
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

    public double getDistance(Pose2D a, Pose2D b) {
        double dx = a.getX(DistanceUnit.CM) - b.getX(DistanceUnit.CM);
        double dy = a.getY(DistanceUnit.CM) - b.getY(DistanceUnit.CM);
        return(Math.sqrt(dx * dx + dy * dy));
    }

    public double getVelocity() {
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
        movePower = 0.3;
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
        double velocity = getVelocity();
        double timeAdjustedDistanceLeft = distanceLeft - velocity * 0.03;
        movePower = Math.min(moveSpeed, movePower + 0.1);
        if (!opMode.opModeIsActive()) {
            moveState = MoveState.STOPPED;
            return;
        }
        if (moveState == MoveState.MOVING) {
            double angularError = odo.getPosition().getHeading(AngleUnit.DEGREES) - desiredHeading;
            angularError = AngleUnit.normalizeDegrees(angularError);
            double adjust = angularError / 40;
            setPowers(movePower + adjust, movePower - adjust, movePower - adjust, movePower + adjust);
            //System.out.println("Angle Error: " + angularError + " Adjust: " + adjust);
            System.out.println("MOVE - Distance Moved: " + distanceMoved + " Distance Left: " + distanceLeft + " Angle Error: " + angularError + " Adjust: " + adjust);
            //System.out.println("Current Braking Distance: " + getBrakingDistance(velocity, forwardBraking));

            if (timeAdjustedDistanceLeft < getStoppingDistance(velocity, forwardCoasting, forwardBraking)) {
                moveState = MoveState.BRAKING;
            }
        } else if (moveState == MoveState.BRAKING) {
            double ratio = velocity / distanceLeft;
            //System.out.println("Current Braking Distance: " + getBrakingDistance(velocity, forwardBraking));
            if ((timeAdjustedDistanceLeft < getBrakingDistance(velocity, forwardBraking)) || (distanceLeft < 0)) {
                System.out.println("MOVE BRAKING - Distance Moved: " + distanceMoved + " Distance Left: " + distanceLeft  + " Velocity: " + getVelocity()  + " Ratio: " + ratio );
                setBraking();
                setPower(0);
            } else {
                System.out.println("MOVE COASTING - Distance Moved: " + distanceMoved + " Distance Left: " + distanceLeft  + " Velocity: " + getVelocity()  + " Ratio: " + ratio );
                setCoasting();
                setPower(0);
            }
            if (getVelocity() < 10) {
                setBraking();
                setPower(0);
                System.out.println("SETTING TO STOPPED");
                System.out.println("X Error: " + (targetEndPose.getX(DistanceUnit.CM) - currentPose.getX(DistanceUnit.CM)) + " Y Error: " + (targetEndPose.getY(DistanceUnit.CM) - currentPose.getY(DistanceUnit.CM)));
                moveState = MoveState.STOPPED;
            }
        }
    }

    public double degreeDifference(double a, double b) {
        return AngleUnit.normalizeDegrees(a-b);
    }
    public double degreeDifference(Pose2D a, Pose2D b) {
        return AngleUnit.normalizeDegrees(getRotation(a)-getRotation(b));
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
        double turnDegrees = degreeDifference(desiredHeading, startRotation);
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
        if (!opMode.opModeIsActive()) {
            moveState = MoveState.STOPPED;
            return;
        }
        if (moveState == MoveState.MOVING) {
            if (degreesLeft < startBraking) {
                moveState = MoveState.BRAKING;
            }
        } else if (moveState == MoveState.BRAKING) {
            double velocity = odo.getHeadingVelocity();
            double ratio = velocity / degreesLeft;
            System.out.println("Turn Braking: " + getBrakingDistance(velocity, turnBraking));
            if ((ratio > 8) || (degreesLeft < 0)) {
                System.out.println("TBRAKING: Velocity: " + velocity + " Degrees Left : " + degreesLeft + " Ratio: " + ratio);
                setBraking();
                setPower(0);
            } else {
                System.out.println("TCOASTING: Velocity: " + velocity + " Degrees Left : " + degreesLeft + " Ratio: " + ratio);
                setCoasting();
                setPower(0);
            }
            if (getVelocity() < 1) {
                setBraking();
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
        if (!opMode.opModeIsActive()) {
            moveState = MoveState.STOPPED;
            return;
        }

        if (moveState == MoveState.MOVING) {
            double angularError = odo.getPosition().getHeading(AngleUnit.DEGREES) - desiredHeading;
            angularError = AngleUnit.normalizeDegrees(angularError);
            double adjust = angularError / 40;
            setPowers(-moveSpeed + adjust, moveSpeed - adjust, -moveSpeed - adjust, moveSpeed + adjust);
            System.out.println("Strafe - Distance Moved: " + distanceMoved + " Distance Left: " + distanceLeft + "Angle Error: " + angularError + " Adjust: " + adjust);
            if (distanceLeft < startBraking) {
                moveState = MoveState.BRAKING;
            }
        } else if (moveState == MoveState.BRAKING) {
            double velocity = getVelocity();
            double ratio = velocity / distanceLeft;
           // System.out.println("Current Braking Distance: " + getBrakingDistance(velocity, strafeBraking));
            if ((ratio > 9.5) || (distanceLeft < 0)) {
                System.out.println("Strafe - BRAKING: Velocity: " + getVelocity() + " Distance Left : " + distanceLeft + " Ratio: " + ratio);
                setBraking();
                setPower(0);
            } else {
                System.out.println("Strafe - COASTING: Velocity: " + getVelocity() + " Distance Left : " + distanceLeft + " Ratio: " + ratio);
                setCoasting();
                setPower(0);
            }
            if (getVelocity() < 1) {
                setBraking();
                setPower(0);
                System.out.println("SETTING TO STOPPED");
                System.out.println("X Error: " + (targetEndPose.getX(DistanceUnit.CM) - currentPose.getX(DistanceUnit.CM)) + " Y Error: " + (targetEndPose.getY(DistanceUnit.CM) - currentPose.getY(DistanceUnit.CM)));
                moveState = MoveState.STOPPED;
            }
        }
    }

    public double getBrakingDistance(double velocity, double[][]data) {
        int datapoints = data.length;
        for (int i = datapoints - 2; i >= 0; i--) {
            if (velocity < data[i][0]) {
                //System.out.println("Time to interpolate: "+ velocity + ", " + data[i][0]+ ", " +
                //h       data[i + 1][0]+ ", " + data[i][1]+ ", " + data[i + 1][1]);
                return interpolate(velocity, data[i][0], data[i + 1][0], data[i][1], data[i + 1][1]);
            }
        }
        return(velocity/data[0][0]*data[0][1]);
    }

    public double interpolate(double v, double v1, double v2, double d1, double d2) {
        double t = (v-v2) / (v1-v2);
        //return (d2 + (d1-d2)* t);
        return interpolate(d1,d2, t);
    }

    public double interpolate (double a, double b, double t) {
        return (b + (a-b)* t);
    }

    public double getStoppingDistance(double velocity, double[][] coasting, double[][] braking) {
        double coastingBrakingDistance = getBrakingDistance(velocity, coasting);
        double brakingBrakingDistance = getBrakingDistance(velocity, braking);
        return interpolate(coastingBrakingDistance, brakingBrakingDistance, 0.5);
    }

    private MoveState figureOutStateStuff(double velocity, double distance, double totalDistance, double[][] coastData, double[][] brakeData) {
        return(MoveState.BRAKING);
    }
}
