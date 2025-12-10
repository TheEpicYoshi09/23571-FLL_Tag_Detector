package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public static final Lift INSTANCE = new Lift();
    private Lift() {  }

    private DcMotorEx lift_motor;
    private static final String motor_name = "lift";

    private static final int STOW_POSITION = 0;
    private static final int TIP_POSITION = 4800;
    private static final double POWER_TO_STOW = -1.;
    private static final double POWER_TO_TIP = +1.;

    public void stow() {
        lift_motor.setTargetPosition(STOW_POSITION);
        lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //lift_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_motor.setPower(POWER_TO_STOW);
    }

    public void hold() {
        lift_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_motor.setPower(0.);
    }

    public void tip() {
        lift_motor.setTargetPosition(TIP_POSITION);
        lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //lift_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_motor.setPower(POWER_TO_TIP);
    }

    public int getPosition() {
        return lift_motor.getCurrentPosition();
    }

    public void init(HardwareMap hMap) {
        lift_motor = hMap.get(DcMotorEx.class, motor_name);
        lift_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_motor.setTargetPosition(0);
    }
}
