

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

//TODO: divide by 25
@TeleOp(name="Robot: Teleop Tank", group="Robot")

public class IdTeleop extends OpMode{

    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftBack     = null;
    public DcMotor  rightBack   = null;

    public CRServo leftIntake = null;
    public CRServo rightIntake = null;
    public DcMotor armExtension = null;
    public DcMotor armRotation = null;
    public Servo wristRotation = null;
    GoBildaPinpointDriver odo;

    private double getVelocity() {
        double x = odo.getVelocity().getX(DistanceUnit.CM);
        double y = odo.getVelocity().getY(DistanceUnit.CM);
        return (Math.sqrt(x * x + y * y));
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
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

        telemetry.addData(">", "Robot Ready.  Press START.");    //

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
//        left = -gamepad1.left_stick_y;
//        right = -gamepad1.right_stick_y;

//        leftFront.setPower(left);
//        leftBack.setPower(left);
//        rightFront.setPower(right);
//        rightBack.setPower(right);
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

        leftFront .setPower(y + x + rx);
        leftBack.setPower(y - x + rx);
        rightFront.setPower(y - x - rx);
        rightBack.setPower(y + x - rx);

        odo.update();
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        double velocity = getVelocity();
        telemetry.addData("Velocity", velocity);
        telemetry.update();



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }



}
