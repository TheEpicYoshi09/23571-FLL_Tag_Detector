package org.firstinspires.ftc.teamcode.AUTO;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.AUTO.Subsystem.Intake;
public class IntakeActions {

    public static double INTAKE_VEL_ALPHA = 0.7;
    public static double INTAKE_DROP_TPS = 200;
    public static long   INTAKE_COOLDOWN_MS = 300;

    private double filtVel = 0;
    private double recentVel = 0;
    private long lastIntakeMs = 0;
    private boolean intakeEvent = false;

    private int ballCount = 0;
    private boolean stopLatched = false;

    public static class takeIn implements Action {
        private final Intake intake;
        private boolean started = false;

        public takeIn(Intake intake) {
            this.intake = intake;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                intake.run();
                started = true;
            } else {
                // keep requesting spinup so triggerHeld stays true
                intake.run();
            }

            intake.update();
            return !intake.consumeIntakeEvent(); // keep running until ready
        }
    }

    public static class stop implements Action {
        private final Intake intake;
        private boolean started = false;

        public stop(Intake intake) {
            this.intake = intake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if ((!started)) {
                intake.stop();
                started = true;
            } else{
                intake.update();
                return false;
            }
            intake.update();
            return !intake.consumeIntakeEvent();
        }
    }
    //Intake

    public static class pushUp implements Action {
        private final Intake intake;

        private boolean started = false;
        private long startTimeMs;

        // how long to run the intake (milliseconds)
        private static final long PUSH_UP_TIME_MS = 1500;

        public pushUp(Intake intake) {
            this.intake = intake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                startTimeMs = System.currentTimeMillis();
                intake.run();
                started = true;
            }

            long elapsed = System.currentTimeMillis() - startTimeMs;

            if (elapsed < PUSH_UP_TIME_MS) {
                intake.run();   // keep pushing
                intake.update();
                return true;    // keep action alive
            } else {
                intake.stop(); // done pushing
                intake.update();
                return false;  // action finished
            }
        }
    }


    //Reverse
    public void resetSession() {
        ballCount = 0;
        stopLatched = false;

        intakeEvent = false;
        lastIntakeMs = 0;
        recentVel = 0;
        //Leaving filtVel as is is probably fine, but can be zeroed too:
        //filtVel = 0;
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
