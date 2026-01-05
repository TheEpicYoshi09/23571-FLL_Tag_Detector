package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotorEx motor;

    public static double INTAKE_VEL_ALPHA = 0.7;
    public static double INTAKE_DROP_TPS = 200;
    public static long   INTAKE_COOLDOWN_MS = 300;

    private double filtVel = 0;
    private double recentVel = 0;
    private long lastIntakeMs = 0;
    private boolean intakeEvent = false;

    private boolean intakingCommanded = false;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void run() {
        motor.setPower(1.0);


        if (!intakingCommanded) {
            intakingCommanded = true;
            intakeEvent = true;
        }
    }

    public void reverse() {
        motor.setPower(-0.2);
        intakingCommanded = false;
    }

    public void stop() {
        motor.setPower(0);
        intakingCommanded = false;
    }

    public void update() {
        double raw = Math.abs(motor.getVelocity());
        filtVel = INTAKE_VEL_ALPHA * raw + (1 - INTAKE_VEL_ALPHA) * filtVel;


        if (motor.getPower() > 0.1) {
            recentVel = Math.max(recentVel, filtVel);

            long now = System.currentTimeMillis();
            boolean drop = (recentVel - filtVel) > INTAKE_DROP_TPS;

            if (drop && (now - lastIntakeMs) > INTAKE_COOLDOWN_MS) {
                intakeEvent = true;
                lastIntakeMs = now;
                recentVel = filtVel;
            }
        } else {
            recentVel = filtVel;
        }
    }

    public boolean consumeIntakeEvent() {
        if (intakeEvent) {
            intakeEvent = false;
            return true;
        }
        return false;
    }
}
