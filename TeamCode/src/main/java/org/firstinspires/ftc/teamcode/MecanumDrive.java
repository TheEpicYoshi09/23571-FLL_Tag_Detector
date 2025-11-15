package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private DcMotor frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor;
    private IMU imu;
    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "motor2");
        backLeftMotor = hwMap.get(DcMotor.class, "motor0");
        frontRightMotor = hwMap.get(DcMotor.class, "motor3");
        backRightMotor = hwMap.get(DcMotor.class, "motor1");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
//e
        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive(double power, double theta, double turn) {

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin),Math.abs(cos));

        double leftFront = power * cos/max + turn;
        double rightFront = power * sin/max - turn;
        double leftRear = power * sin/max + turn;
        double rightRear = power * cos/max - turn;

        if ((power + Math.abs(turn)) > 1){
            leftFront /= power + Math.abs(turn);
            rightFront /= power + Math.abs(turn);
            leftRear /= power + Math.abs(turn);
            rightRear /= power + Math.abs(turn);
        }

        frontLeftMotor.setPower(leftFront);
        frontRightMotor.setPower(rightFront);
        backRightMotor.setPower(rightRear);
        backLeftMotor.setPower(leftRear);
    }

    public void driveFieldRelative(double y, double x, double turn){
        double theta = Math.atan2(y,x);
        double r = Math.hypot(x,y);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        this.drive(r,theta,turn);
    }

    public void driveRobotRelative(double y, double x, double turn){
        double theta = Math.atan2(y,x);
        double r = Math.hypot(x,y);

        this.drive(r,theta,turn);
    }
}
