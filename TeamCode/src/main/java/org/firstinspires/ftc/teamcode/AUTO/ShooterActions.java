package org.firstinspires.ftc.teamcode.AUTO;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AUTO.Subsystem.Shooter;

public class ShooterActions {

    // Spins up and finishes once shooter reports READY
    public static class SpinUpUntilReady implements Action {
        private final Shooter shooter;
        private boolean started = false;

        public SpinUpUntilReady(Shooter shooter) {
            this.shooter = shooter;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                shooter.spinUp();
                started = true;
            } else {
                // keep requesting spinup so triggerHeld stays true
                shooter.spinUp();
            }

            shooter.update();

            packet.put("ShooterReady", shooter.isReady());
            packet.put("ShooterRPM", shooter.getRPM());

            return !shooter.isReady(); // keep running until ready
        }
    }

    // Holds spin for a fixed time (useful to keep it alive while driving)
    public static class HoldSpinForTime implements Action {
        private final Shooter shooter;
        private final double seconds;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean started = false;

        public HoldSpinForTime(Shooter shooter, double seconds) {
            this.shooter = shooter;
            this.seconds = seconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                timer.reset();
                started = true;
            }

            shooter.spinUp();
            shooter.update();

            packet.put("ShooterRPM", shooter.getRPM());

            return timer.seconds() < seconds;
        }
    }

    // Fires once. Uses time instead of shotEvent so it still completes even if no ball.
    public static class FireOnceTimed implements Action {
        private final Shooter shooter;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean started = false;

        public FireOnceTimed(Shooter shooter) {
            this.shooter = shooter;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                shooter.fire();
                timer.reset();
                started = true;
            }

            shooter.update();

            // Finish after transfer window + small buffer
            double doneAfterSec = (Shooter.TRANSFER_TOTAL_DURATION_MS + 100) / 1000.0;
            return timer.seconds() < doneAfterSec;
        }
    }

    // Hard stop
    public static class Stop implements Action {
        private final Shooter shooter;
        private boolean done = false;

        public Stop(Shooter shooter) {
            this.shooter = shooter;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!done) {
                shooter.abort();
                shooter.update();
                done = true;
            }
            return false;
        }
    }
}

