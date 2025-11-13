package org.firstinspires.ftc.teamcode.subsystems;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

public class Shooting extends Subsystem {
    private Shooting() {
    }

    public static final Shooting INSTANCE = new Shooting();

    //USER CODE
    public String motor_name1 = "shooting_Motor_Left";
    public String motor_name2 = "shooting_Motor_Right";
    private final double SHOOTING_POWER_IN = 1.;
    private final double SHOOTING_POWER_OUT = -.9;
    private final double SHOOTING_POWER_OFF = 0.;

    public MotorEx motor;



    @Override
    public void initialize() {
        motor = new MotorEx(motor_name1);
        motor = new MotorEx(motor_name2);
    }

}