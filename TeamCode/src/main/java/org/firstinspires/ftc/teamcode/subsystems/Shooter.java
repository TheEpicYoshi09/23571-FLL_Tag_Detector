package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

public class Shooter {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }

    private DcMotorEx left_motor = null;
    private DcMotorEx right_motor = null;
    private CRServo kicker = null;

    private String left_name = "left_shooter";
    private String right_name = "right_shooter";
    private String servo_name = "kicker";


    private static double POWER_TO_LAUNCH = -.50;
    private static double SHOOTER_SPEED_LOW = -.25;
    private static double SHOOTER_SPEED_MED = -.50;
    private static double SHOOTER_SPEED_HIGH = .75;
    private static double POWER_TO_BACK = -.25;

    public void back() {
        left_motor.setPower(POWER_TO_BACK);
        right_motor.setPower(POWER_TO_BACK);
    }

    public void launch() {
        left_motor.setPower(POWER_TO_LAUNCH);
        right_motor.setPower(POWER_TO_LAUNCH);
    }
    public void medium() {
        left_motor.setPower(SHOOTER_SPEED_MED);
        right_motor.setPower(SHOOTER_SPEED_MED);
    }

    public void low() {
      left_motor.setPower(SHOOTER_SPEED_LOW);
      right_motor.setPower(SHOOTER_SPEED_LOW);
    }

    public void high() {
        left_motor.setPower(SHOOTER_SPEED_HIGH);
        right_motor.setPower(SHOOTER_SPEED_HIGH);
    }

    public void stop() {
        left_motor.setPower(0.);
        right_motor.setPower(0.);
    }

    public double getSpeed() {
        return left_motor.getVelocity();
    }

    public void kickeron() {
        kicker.setPower(0.5);
    }
    public void kickerout() {
        kicker.setPower(-0.5);
    }
    public void kickeroff() {
        kicker.setPower(0.);
    }

    public void init(HardwareMap hMap) {
        kicker = hMap.get(CRServo.class,servo_name);
        left_motor = hMap.get(DcMotorEx.class,left_name);
        right_motor = hMap.get(DcMotorEx.class,right_name);
        left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stop();
    }

}

