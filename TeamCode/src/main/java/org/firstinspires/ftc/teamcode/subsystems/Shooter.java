package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Shooter {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }

    private DcMotorEx left_motor = null;
    private DcMotorEx right_motor = null;
    private Servo kicker = null;

    private String left_name = "left_shooter";
    private String right_name = "right_shooter";
    private String servo_name = "kicker";

    private static double POWER_TO_LAUNCH = -.80;
    private static double POWER_TO_BACK = -.25;

    public void back() {
        left_motor.setPower(POWER_TO_BACK);
        right_motor.setPower(POWER_TO_BACK);
    }

    public void launch() {
        left_motor.setPower(POWER_TO_LAUNCH);
        right_motor.setPower(POWER_TO_LAUNCH);
    }

    public void stop() {
        left_motor.setPower(0.);
        right_motor.setPower(0.);
    }

    public void kickeron() {
        kicker.setSpeed(0.5);
    }
    public void kickeron() {
        kicker.setSpeed(0.);
    }

    public void init(HardwareMap hmap) {
        kicker = hmap.get(DcMotorEx.class,servo_name);
        left_motor = hmap.get(DcMotorEx.class,left_name);
        right_motor = hmap.get(DcMotorEx.class,left_name);
        left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.COAST);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.COAST);
        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	this.stop()
        timer = new ElapsedTime();
        timer.reset();
    }

}

