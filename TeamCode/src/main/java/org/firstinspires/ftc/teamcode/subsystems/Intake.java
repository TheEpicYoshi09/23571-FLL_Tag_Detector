package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    private String motor_name = "intake_motor";
    public DcMotorEx intake_motor;

    private double INTAKE_OUT_POWER = 0.59;
    private double INTAKE_IN_POWER = -0.80;
    private double INTAKE_OFF_POWER = 0.0;
    private double INTAKE_MIN_POWER = -0.2;

    public void intakein(){
        intake_motor.setPower(INTAKE_IN_POWER);
    }
    public void intakeout(){
        intake_motor.setPower(INTAKE_OUT_POWER);
    }
    public void intakeoff() {
        double minSpeed = 0.;
        if (Drive.INSTANCE.fwdSpeed > 0.1) {
            minSpeed = INTAKE_MIN_POWER;
        }
        intake_motor.setPower(minSpeed);
    }

    public void init(HardwareMap hMap) {
        intake_motor = hMap.get(DcMotorEx.class, motor_name);

        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
