package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
public class Shooter {

    public Action spinUpAction() {
        return new SpinUpAction();
    }

    public Action fireAction() {
        return new FireAction();
    }

    private class SpinUpAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            spinUp();
            update();
            return !isReady();
        }
    }

    private class FireAction implements Action {
        private boolean fired = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!fired) {
                fire();
                fired = true;
            }
            update();
            return !consumeShotEvent();
        }
    }
    public enum State {
        IDLE,
        SPINNING_UP,
        READY,
        FIRING
    } //different states of the shooter


    public static double VELOCITY_ALPHA = 0.7;
    public static double VOLTAGE_ALPHA = 0.3;


    public static double kS = 0.3;
    public static double kV = 0.0019;
    public static double kP = 0.004;


    public static double TARGET_RPM = 4900;


    public static double COAST_CONSTANT = 0.025;
    public static double READY_THRESHOLD = 100;


    public static double TRANSFER_START_TO_CONTACT_MS = 150;
    public static double BALL_CONTACT_DURATION_MS = 80;
    public static double TRANSFER_TOTAL_DURATION_MS = 300;
    public static double BOOST_BUFFER_MS = 20;
    public static double SPINUP_TIMEOUT_MS = 2000;


    private static final double MIN_SAFE_VOLTAGE = 10.0;


    public static double SHOT_DROP_RPM = 250;
    public static double SHOT_DROP_ACCEL = -6000;
    public static long   SHOT_COOLDOWN_MS = 250;


    public static double MANUAL_BACKFEED_MOTOR_POWER = -0.7;


    public static long TRANSFER_PULSE_MS = 140;

    private final DcMotorEx shooter;
    private final CRServo transferTop;
    private final CRServo transferBottom;
    private final VoltageSensor voltageSensor;

    private State state = State.IDLE;

    private double filteredRPM = 0;
    private double lastFilteredRPM = 0;
    private double filteredVoltage = 12.0;
    private double acceleration = 0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime spinUpTimer = new ElapsedTime();

    private boolean triggerHeld = false;


    private boolean shotEvent = false;
    private double recentPeakRpm = 0;
    private long lastShotMs = 0;


    private boolean manualBackfeed = false;


    private long transferPulseUntilMs = 0;

    public Shooter(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        shooter = hardwareMap.get(DcMotorEx.class, "sr");
        transferTop = hardwareMap.get(CRServo.class, "tl");
        transferBottom = hardwareMap.get(CRServo.class, "bl");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        loopTimer.reset();
    } //typical, blah blah blah

//WHERE THINGS START GETTING COOL:
    public void setManualBackfeed(boolean enabled) {
        manualBackfeed = enabled;
    }

    //Pulse system instead of holding button
    public void pulseTransferForward() {
        transferPulseUntilMs = System.currentTimeMillis() + TRANSFER_PULSE_MS;
    }

    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0) dt = 0.02;

        long nowMs = System.currentTimeMillis();


        if (manualBackfeed) {
            setTransfer(-1);
            shooter.setPower(MANUAL_BACKFEED_MOTOR_POWER);
            return;
        }


        double rawRPM = Math.abs((shooter.getVelocity() / 28.0) * 60.0);
        filteredRPM = (VELOCITY_ALPHA * rawRPM) + ((1 - VELOCITY_ALPHA) * filteredRPM);

        double rawVoltage = voltageSensor.getVoltage();
        filteredVoltage = (VOLTAGE_ALPHA * rawVoltage) + ((1 - VOLTAGE_ALPHA) * filteredVoltage);

        acceleration = (filteredRPM - lastFilteredRPM) / dt;
        lastFilteredRPM = filteredRPM;

        double error = TARGET_RPM - filteredRPM;
        double desiredVolts = 0;

//Tracks recent peak whenever not idle
        if (state != State.IDLE) {
            recentPeakRpm = Math.max(recentPeakRpm, filteredRPM);
        } else {
            recentPeakRpm = filteredRPM;
        }


        double transferCmd = 0;


        switch (state) {
            case IDLE:
                desiredVolts = 0;
                transferCmd = 0;
                break;

            case SPINNING_UP:
                desiredVolts = filteredVoltage;
                transferCmd = 0;

                if (spinUpTimer.milliseconds() > SPINUP_TIMEOUT_MS) {
                    state = State.IDLE;
                    break;
                }

                double predictedFinal = filteredRPM + (acceleration * COAST_CONSTANT);
                if (predictedFinal >= TARGET_RPM) {
                    state = State.READY;
                }
                break;

            case READY:
                desiredVolts = kS + (kV * TARGET_RPM) + (kP * error);
                transferCmd = 0;

                if (Math.abs(error) > READY_THRESHOLD * 3) {
                    state = State.SPINNING_UP;
                    spinUpTimer.reset();
                }
                break;

            case FIRING:
                double t = shotTimer.milliseconds();

                double contactStart = TRANSFER_START_TO_CONTACT_MS;
                double contactEnd = contactStart + BALL_CONTACT_DURATION_MS;
                double boostStart = contactStart - BOOST_BUFFER_MS;

                transferCmd = (t < TRANSFER_TOTAL_DURATION_MS) ? 1 : 0;

                if (t < boostStart) {
                    desiredVolts = kS + (kV * TARGET_RPM) + (kP * error);
                } else if (t < contactEnd) {
                    desiredVolts = filteredVoltage;
                } else {
                    double predictedRecovery = filteredRPM + (acceleration * COAST_CONSTANT);

                    if (predictedRecovery >= TARGET_RPM) {
                        desiredVolts = kS + (kV * TARGET_RPM) + (kP * error);

                        if (Math.abs(error) < READY_THRESHOLD) {
                            if (triggerHeld) {
                                shotTimer.reset();
                            } else {
                                state = State.READY;
                            }
                        }
                    } else {
                        desiredVolts = filteredVoltage;
                    }
                }
                break;
        }


        if (state != State.FIRING && nowMs < transferPulseUntilMs) {
            transferCmd = 1;
        }


        setTransfer(transferCmd);


        if (state != State.IDLE) {
            boolean bigDrop = (recentPeakRpm - filteredRPM) > SHOT_DROP_RPM;
            boolean hardDecel = acceleration < SHOT_DROP_ACCEL;

            if (bigDrop && hardDecel && (nowMs - lastShotMs) > SHOT_COOLDOWN_MS) {
                shotEvent = true;
                lastShotMs = nowMs;
                recentPeakRpm = filteredRPM;
            }
        }


        double safeVoltage = Math.max(filteredVoltage, MIN_SAFE_VOLTAGE);
        double power = safeClamp(desiredVolts / safeVoltage, 0, 1);
        shooter.setPower(power);
    }

    private double safeClamp(double value, double min, double max) {
        if (Double.isNaN(value) || Double.isInfinite(value)) return 0.0;
        return Math.max(min, Math.min(value, max));
    }

    public void spinUp() {
        if (state == State.IDLE) {
            state = State.SPINNING_UP;
            spinUpTimer.reset();
        }
        triggerHeld = true;
    }

    public void abort() {
        state = State.IDLE;
        triggerHeld = false;
        setTransfer(0);
    }

    public void setTriggerHeld(boolean held) {
        triggerHeld = held;
        if (!held && state != State.FIRING) {
            state = State.IDLE;
        }
    }

    public void fire() {
        if (state == State.READY) {
            state = State.FIRING;
            shotTimer.reset();
        }
    }


    private void setTransfer(double speed) {
        if (speed > 0) {
            transferTop.setPower(1.0);
            transferBottom.setPower(-1.0);
        } else if (speed < 0) {
            transferTop.setPower(-1.0);
            transferBottom.setPower(1.0);
        } else {
            transferTop.setPower(0);
            transferBottom.setPower(0);
        }
    }

    public boolean consumeShotEvent() {
        if (shotEvent) {
            shotEvent = false;
            return true;
        }
        return false;
    }

    public boolean isReady() {
        return state == State.READY;
    }

    public double getRPM() {
        return filteredRPM;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public double getVoltage() {
        return filteredVoltage;
    }
}
