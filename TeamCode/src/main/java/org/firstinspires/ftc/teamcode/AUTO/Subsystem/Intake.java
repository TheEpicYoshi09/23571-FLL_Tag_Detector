package org.firstinspires.ftc.teamcode.AUTO.Subsystem;


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

    private int ballCount = 0;
    private boolean stopLatched = false;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Intake
    public void run() {
        if (stopLatched) {
            motor.setPower(0);
        } else {
            motor.setPower(0.8);
        }
    }

    //Reverse
    public void reverse() {
        motor.setPower(-1);
    }

    public void stop() {
        motor.setPower(0);
    }

    public void resetSession() {
        ballCount = 0;
        stopLatched = false;

        intakeEvent = false;
        lastIntakeMs = 0;
        recentVel = 0;
        //Leaving filtVel as is is probably fine, but can be zeroed too:
        //filtVel = 0;
    }

    public void update() {
        double raw = Math.abs(motor.getVelocity());
        filtVel = INTAKE_VEL_ALPHA * raw + (1 - INTAKE_VEL_ALPHA) * filtVel;

        if (!stopLatched && motor.getPower() > 0.1) {
            recentVel = Math.max(recentVel, filtVel);

            long now = System.currentTimeMillis();
            boolean drop = (recentVel - filtVel) > INTAKE_DROP_TPS;

            if (drop && (now - lastIntakeMs) > INTAKE_COOLDOWN_MS) {
                intakeEvent = true;
                lastIntakeMs = now;
                recentVel = filtVel;

                ballCount++;

                //Ful stop
                if (ballCount >= 3) {
                    stopLatched = true;
                    motor.setPower(0);
                }
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
    public int getBallCount() { return ballCount; }
    public boolean isStopLatched() { return stopLatched; }
}
