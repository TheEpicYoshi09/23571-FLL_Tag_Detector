package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class ShooterSubsystem {
    public enum State { IDLE, PRIME_INTAKE, PRIME_OUTTAKE, SPIN_UP, FIRE_BALLS, SPIN_DOWN_ALL }

    // Angling servos (not important per your note)
    private static final double ANGLE_MIN = 0.35;
    private static final double ANGLE_MAX = 0.9;
    private static final double ANGLE_STEP = 0.02;
    private double anglePos = 0.5;

    // Hardware
    private final DcMotor intake;
    private final Servo intakeBlockServo;
    private final DcMotorEx outtakeMotor;
    private final Servo leftVerticalServo;

    // Tunables
    private final int shortShotVelocity = 1400;
    private final int longShotVelocity = 2000;
    private final int ejectVelocity = 1000;
    private final double intakePower = 1.0;
    private final long primeIntakeMs = 900;
    private final long primeOuttakeMs = 300;
    private final long spinUpMs = 2000;
    private final long feedMs = 1000;
    private final long spinDownMs = 1000;
    private final int velocityTolerance = 35;

    private final ElapsedTime timer = new ElapsedTime();
    private State state = State.IDLE;
    private boolean busy = false;

    private int numberOfShots = 0;     // remaining shots to fire
    private String shotType = "";      // "short", "long", or "eject"
    private double lastVelocity = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeBlockServo = hardwareMap.get(Servo.class, "intakeBlockServo");
        leftVerticalServo = hardwareMap.get(Servo.class, "leftVerticalServo");

        outtakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        outtakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intakeBlockServo.setDirection(Servo.Direction.REVERSE);
        leftVerticalServo.setDirection(Servo.Direction.REVERSE);

        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ---------- State entry helper ----------
    private void enterState(State newState) {
        state = newState;
        timer.reset();
    }

    /** Start a multi-shot sequence. Non-blocking. */
    public void startShot(int shots, String type) {
        if (busy) return;

        numberOfShots = Math.max(0, shots);
        shotType = type == null ? "" : type;
        busy = true;

        // Begin by priming intake (same as new code)
        intake.setPower(-0.5);
        enterState(State.PRIME_INTAKE);
    }

    /** Eject a number of balls: spins to eject velocity then feeds. */
    public void eject(int balls) {
        if (busy) return;
        numberOfShots = Math.max(0, balls);
        shotType = "eject";
        busy = true;

        // Start spin-up for eject sequence
        outtakeMotor.setVelocity(ejectVelocity);
        enterState(State.SPIN_UP);
    }

    /** Call this every loop to advance the sequence without blocking. */
    public void update() {
        lastVelocity = outtakeMotor.getVelocity();

        switch (state) {

            case IDLE:
                // Ensure motors are safe stopped in idle
                intake.setPower(0);
                // outtakeMotor left to hold last setting or zero; keep as-is
                break;

            case PRIME_INTAKE:
                // Run intake backwards briefly to seat the ball
                intake.setPower(-0.5);
                if (timer.milliseconds() >= primeIntakeMs) {
                    intake.setPower(0);
                    // gentle reverse bump of shooter to settle (optional)
                    outtakeMotor.setPower(-0.2);
                    enterState(State.PRIME_OUTTAKE);
                }
                break;

            case PRIME_OUTTAKE:
                // brief bump window
                if (timer.milliseconds() >= primeOuttakeMs) {
                    outtakeMotor.setPower(0);
                    enterState(State.SPIN_UP);

                    // If shotType is already set to short/long/eject, set target velocity now
                    if (Objects.equals(shotType, "short")) {
                        outtakeMotor.setVelocity(shortShotVelocity);
                    } else if (Objects.equals(shotType, "long")) {
                        outtakeMotor.setVelocity(longShotVelocity);
                    } else if (Objects.equals(shotType, "eject")) {
                        outtakeMotor.setVelocity(ejectVelocity);
                    } else {
                        // default to long if not specified
                        outtakeMotor.setVelocity(longShotVelocity);
                    }
                }
                break;

            case SPIN_UP:
                // Wait until wheel velocity reaches target (or timeout) then feed
                int targetVel;
                if (Objects.equals(shotType, "short")) {
                    targetVel = shortShotVelocity;
                } else if (Objects.equals(shotType, "eject")) {
                    targetVel = ejectVelocity;
                } else {
                    targetVel = longShotVelocity;
                }

                double velError = Math.abs(targetVel - lastVelocity);

                if (velError <= velocityTolerance || timer.milliseconds() >= spinUpMs) {
                    // prepare feeding: mimic old behavior where short vs long used different blocker calls
                    if (Objects.equals(shotType, "short")) {
                        unBlockIntake();   // old code unblocked intake for short
                    } else {
                        blockIntake();     // old code blocked intake for long (and eject used feed directly)
                    }
                    timer.reset();
                    intake.setPower(intakePower); // push ball into shooter
                    enterState(State.FIRE_BALLS);
                }
                break;

            case FIRE_BALLS:
                // feeding window: run intake for feedMs then stop and spin down
                if (timer.milliseconds() >= feedMs) {
                    intake.setPower(0);
                    outtakeMotor.setVelocity(0); // stop flywheel for spin-down window
                    enterState(State.SPIN_DOWN_ALL);
                }
                break;

            case SPIN_DOWN_ALL:
                // If more shots remain, spin back up and decrement shots (old logic)
                if (numberOfShots > 1) {
                    numberOfShots--;
                    enterState(State.SPIN_UP);

                    if (Objects.equals(shotType, "short")) {
                        outtakeMotor.setVelocity(shortShotVelocity);
                    } else if (Objects.equals(shotType, "eject")) {
                        outtakeMotor.setVelocity(ejectVelocity);
                    } else {
                        outtakeMotor.setVelocity(longShotVelocity);
                    }
                } else {
                    // final spin-down timeout -> go to idle
                    if (timer.milliseconds() >= spinDownMs) {
                        enterState(State.IDLE);
                        busy = false;
                    }
                }
                break;
        }
    }

    // Intake blockers (keeps your API)
    public void blockIntake() {
        // caller said don't worry servos; these positions are placeholders and should be tuned
        intakeBlockServo.setPosition(0.0);
    }

    public void unBlockIntake() {
        intakeBlockServo.setPosition(1.0);
    }

    // --- Manual angling (unchanged) ---
    public void manualAngling(double leftStickY) {
        double input = -leftStickY;
        if (Math.abs(input) < 0.08) return;
        anglePos += input * ANGLE_STEP;
        anglePos = Math.max(ANGLE_MIN, Math.min(ANGLE_MAX, anglePos));
        leftVerticalServo.setPosition(anglePos);
    }

    public void angleUp() {
        anglePos = Math.min(anglePos + ANGLE_STEP, ANGLE_MAX);
        leftVerticalServo.setPosition(anglePos);
    }

    public void angleDown() {
        anglePos = Math.max(anglePos - ANGLE_STEP, ANGLE_MIN);
        leftVerticalServo.setPosition(anglePos);
    }

    public void setAngle(double position) {
        anglePos = Math.max(ANGLE_MIN, Math.min(ANGLE_MAX, position));
        leftVerticalServo.setPosition(anglePos);
    }

    public double getAngle() { return anglePos; }

    // Intake/outtake direct controls
    public void startIntake(double power) { intake.setPower(power); }
    public void reverseIntake(double power) { intake.setPower(-power); }
    public void stopIntake() { intake.setPower(0); }

    public void startOuttake(double power) { outtakeMotor.setPower(power); }
    public void stopOuttake() { outtakeMotor.setPower(0); }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- Shooter -----");
        telemetry.addData("Shooter Velocity", lastVelocity);
        telemetry.addData("State", state.toString());
        telemetry.addData("Remaining Shots", numberOfShots);
        telemetry.addData("Vertical Aim Pos", anglePos);
    }

    public boolean isBusy() { return busy; }
    public boolean isNotBusy() { return !busy; }
    public State getState() { return state; }
}
