package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class SpindexerController {
    private final RobotHardware robot;
    private final ArtifactTracker artifactTracker;
    private final Telemetry telemetry;

    private final double[] spindexerPositions = new double[]{Constants.spindexer1, Constants.spindexer2, Constants.spindexer3};
    private int spindexerIndex = 0;
    private double spindexerPos = Constants.spindexerStart;

    private boolean spindexerFull = false;
    private boolean autoSpinEnabled = false;

    private long sensorReadAllowedAtNanos = 0L;
    private long autoRotateAllowedAtNanos = 0L;

    private static final long SENSOR_SETTLE_TIME_NANOS = 500_000_000L; // 0.5 seconds
    private final ArtifactTracker.SlotStatus[] cachedSlots = new ArtifactTracker.SlotStatus[]{
            ArtifactTracker.SlotStatus.VACANT,
            ArtifactTracker.SlotStatus.VACANT,
            ArtifactTracker.SlotStatus.VACANT
    };

    public SpindexerController(RobotHardware robot, ArtifactTracker artifactTracker, Telemetry telemetry) {
        this.robot = robot;
        this.artifactTracker = artifactTracker;
        this.telemetry = telemetry;
    }

    public void init() {
        setPosition(0);
    }

    public void update() {
        telemetry.addLine("--- SPINDEXER ---");

        if (autoSpinEnabled) {
            telemetry.addLine("AUTO SPINDEXER ENABLED!");

            boolean allowSensorRead = System.nanoTime() >= sensorReadAllowedAtNanos;
            boolean allowAutoRotation = System.nanoTime() >= autoRotateAllowedAtNanos;
            ArtifactTracker.SlotStatus[] slots = cachedSlots;

            if (allowSensorRead) {
                ArtifactTracker.SlotStatus[] latest = artifactTracker.getSlotStatuses();
                System.arraycopy(latest, 0, cachedSlots, 0, cachedSlots.length);
                slots = cachedSlots;
            } else {
                telemetry.addLine("Spindexer settling; holding position");
            }

            spindexerFull = allSlotsFilled(slots);
            if (spindexerFull) {
                if (allowAutoRotation) {
                    setPosition(0);
                    telemetry.addData("SPINDEXER POSITION", spindexerPos);
                    telemetry.addData("SPINDEXER INDEX", spindexerIndex);
                    telemetry.addLine("SPINDEXER FULL!");
                    telemetry.addLine("-------------------------");
                }
                return;
            }

            if (!allowSensorRead) {
                telemetry.addData("SPINDEXER POSITION", spindexerPos);
                telemetry.addData("SPINDEXER INDEX", spindexerIndex);
                telemetry.addLine("-------------------------");
                return;
            }

            if (getIndex() == 0) {
                boolean hiddenSlotVacant = slots[2] == ArtifactTracker.SlotStatus.VACANT;
                if (hiddenSlotVacant && slotsFilled(slots, 0, 1)) {
                    if (allowAutoRotation) {
                        setPosition(1);
                    }
                }
            } else {
                boolean hiddenSlotVacant = slots[0] == ArtifactTracker.SlotStatus.VACANT;
                if (hiddenSlotVacant && slotsFilled(slots, 1, 2)) {
                    if (allowAutoRotation) {
                        setPosition(0);
                    }
                }
            }
        }

        telemetry.addData("SPINDEXER POSITION", spindexerPos);
        telemetry.addData("SPINDEXER INDEX", spindexerIndex);
        telemetry.addLine("-------------------------");
    }

    public void toggleAuto() {
        autoSpinEnabled = !autoSpinEnabled;
    }

    public boolean isEnabled() {
        return autoSpinEnabled;
    }
    public boolean isSpindexerFull() {
        return spindexerFull;
    }

    private boolean allSlotsFilled(ArtifactTracker.SlotStatus[] slots) {
        for (ArtifactTracker.SlotStatus status : slots) {
            if (status == ArtifactTracker.SlotStatus.VACANT) {
                return false;
            }
        }
        return true;
    }

    private boolean slotsFilled(ArtifactTracker.SlotStatus[] slots, int firstIndex, int secondIndex) {
        return slots[firstIndex] != ArtifactTracker.SlotStatus.VACANT
                && slots[secondIndex] != ArtifactTracker.SlotStatus.VACANT;
    }

    public void setPosition(int index) {
        int newIndex = Math.floorMod(index, 3);
        if (spindexerIndex == newIndex) return; // this probably does nothing but im hoping it'll stop it from trying to set the position to the current position, and draw less power or something????????
        spindexerIndex = newIndex;
        robot.spindexer.setPosition(spindexerPositions[spindexerIndex]);
        spindexerPos = spindexerPositions[spindexerIndex];
    }

    public double getPosition() {
        return spindexerPositions[spindexerIndex];
    }

    public int getIndex() {
        return spindexerIndex;
    }

    public void resetPosition() {
        setPosition(0);
    }
    public void advanceSpindexer() {
        setPosition(spindexerIndex + 1);
    }

    public void reverseSpindexer() {
        setPosition(spindexerIndex - 1);
    }

    public void testAdvanceSpindexer(double delta) {
        spindexerPos += delta;
        spindexerPos = Math.max(0.0, Math.min(1.0, spindexerPos));
        robot.spindexer.setPosition(spindexerPos);
    }

    public void testReverseSpindexer(double delta) {
        spindexerPos += delta;
        spindexerPos = Math.max(0.0, Math.min(1.0, spindexerPos));
        robot.spindexer.setPosition(spindexerPos);
    }
}
