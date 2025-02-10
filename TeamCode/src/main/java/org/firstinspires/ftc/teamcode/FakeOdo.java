package org.firstinspires.ftc.teamcode;


import android.view.DragEvent;

import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.security.cert.CertificateEncodingException;

public class  FakeOdo  {
    public Pose2D zeroPose = new Pose2D(DistanceUnit.CM,0,0, AngleUnit.DEGREES, 0);
    OpMode opMode;
    public DcMotor leftFront, rightFront, leftBack, rightBack;
    public int leftFrontPosition, rightFrontPosition, leftBackPosition, rightBackPosition;
    public IMU imu;
    int LF = 0;
    int RF = 1;
    int LB = 2;
    int RB = 3;
    int[] positions = new int[4];

    int[] motorNumbers = {0,1,2,3};
    public DcMotor motors[] = new DcMotor[4];
    public Pose2D currentPose = zeroPose;
    public Pose2D currentDelta = zeroPose;


    FakeOdo() {
    }

    public void init(IDRobot robot){
        opMode = robot.opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;
        motors[LF]=robot.leftFront;
        motors[RF]=robot.rightFront;
        motors[LB]=robot.leftBack;
        motors[RB]=robot.rightBack;
        for (int i: motorNumbers) {
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )

                )
        );
    }

    public Pose2D getVelocity() {
        return currentDelta;
    }

    public void setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods goBildaOdometryPods) {
    }

    public void setOffsets(int i, int i1) {
    }

    public void setEncoderDirections(GoBildaPinpointDriver.EncoderDirection encoderDirection, GoBildaPinpointDriver.EncoderDirection encoderDirection1) {
    }

    public void resetPosAndIMU() {
    }

    public void update() {
        //int[] positionsNow = new int[4];
        int[] deltas = new int[4];
        for (int i: motorNumbers) {
            int position = motors[i].getCurrentPosition();
            deltas[i] = position - positions[i];
            positions[i]= position;
        }
        double deltaForward = (deltas[0] + deltas[1] + deltas[2] + deltas[3])/4.0;
        double deltaRight =  (deltas[LF] - deltas[RF] + deltas[RB] - deltas[LB] )/4.0;
        double angle = imu.getRobotYawPitchRollAngles().getYaw();
        double dx = deltaForward * Math.cos(Math.toRadians(angle)) + deltaRight * Math.sin(Math.toRadians(angle));
        double dy = deltaForward * Math.sin(Math.toRadians(angle)) + deltaRight * Math.cos(Math.toRadians(angle));
        currentDelta  = new Pose2D(DistanceUnit.CM,dx,dy,AngleUnit.DEGREES,AngleUnit.normalizeDegrees(angle - currentPose.getHeading(AngleUnit.DEGREES)));
        double x = currentPose.getX(DistanceUnit.CM) + dx;
        double y = currentPose.getY(DistanceUnit.CM) + dy;
        currentPose = new Pose2D(DistanceUnit.CM,x,y,AngleUnit.DEGREES,angle);
    }

    public Pose2D getPosition() {
        return (currentPose);
    }

    public void setPosition(Pose2D zeroPose) {
    }

    public double getHeadingVelocity() {
        return(imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate);
    }

    public double getHeading() {
        return(imu.getRobotYawPitchRollAngles().getYaw());
    }

    public double getPosX() {
        return currentPose.getX(DistanceUnit.MM);
    }
    public double getPosY() {
        return currentPose.getY(DistanceUnit.MM);
    }

}
