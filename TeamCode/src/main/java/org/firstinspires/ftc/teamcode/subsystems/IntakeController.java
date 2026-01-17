package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Coordinates intake actions with the spindexer and ArtifactTracker color
 * sensors. The controller keeps at least one open slot pointed toward the
 * intake by rotating the spindexer when both visible slots are occupied.
 */
public class IntakeController {

    private final RobotHardware robot;
    private final ArtifactTracker artifactTracker;
    private final SpindexerController spindexerController;
    private final Telemetry telemetry;

    private boolean spindexerFull = false;

    private final ArtifactTracker.SlotStatus[] cachedSlots = new ArtifactTracker.SlotStatus[]{
            ArtifactTracker.SlotStatus.VACANT,
            ArtifactTracker.SlotStatus.VACANT,
            ArtifactTracker.SlotStatus.VACANT
    };
    private long sensorReadAllowedAtNanos = 0L;
    private long autoRotateAllowedAtNanos = 0L;

    private static final long SENSOR_SETTLE_TIME_NANOS = 500_000_000L; // 0.5 seconds

    public IntakeController(RobotHardware robot, ArtifactTracker artifactTracker, SpindexerController spindexerController, Telemetry telemetry) {
        this.robot = robot;
        this.artifactTracker = artifactTracker;
        this.spindexerController = spindexerController;
        this.telemetry = telemetry;
    }

    /**
     * Refresh sensor classifications and rotate the spindexer whenever both
     * slots facing the intake are occupied. When all three slots contain
     * artifacts, the spindexer is returned to position 1.
     */
    public void update() {
        boolean allowSensorRead = System.nanoTime() >= sensorReadAllowedAtNanos;
        boolean allowAutoRotation = System.nanoTime() >= autoRotateAllowedAtNanos;
        ArtifactTracker.SlotStatus[] slots = cachedSlots;

        if (allowSensorRead) {
            artifactTracker.update();
            ArtifactTracker.SlotStatus[] latest = artifactTracker.getSlotStatuses();
            System.arraycopy(latest, 0, cachedSlots, 0, cachedSlots.length);
            slots = cachedSlots;
        } else {
            telemetry.addLine("Spindexer settling; holding position");
        }

        spindexerFull = allSlotsFilled(slots);
        if (spindexerFull) {
            if (allowAutoRotation) {
                moveToPosition(0);
                telemetry.addLine("Spindexer full; returned to position 1");
            } else {
                telemetry.addLine("Spindexer full; waiting to rotate to position 1");
            }
            return;
        }

        if (!allowSensorRead) {
            telemetry.addData("Spindexer Facing", spindexerController.getIndex() + 1);
            return;
        }

        if (spindexerController.getIndex() == 0) {
            boolean hiddenSlotVacant = slots[2] == ArtifactTracker.SlotStatus.VACANT;
            if (hiddenSlotVacant && slotsFilled(slots, 0, 1)) {
                if (allowAutoRotation) {
                    moveToPosition(1);
                    telemetry.addLine("Slots 1 & 2 full; rotating to position 2");
                } else {
                    telemetry.addLine("Slots 1 & 2 full; waiting to rotate to position 2");
                }
            }
        } else {
            boolean hiddenSlotVacant = slots[0] == ArtifactTracker.SlotStatus.VACANT;
            if (hiddenSlotVacant && slotsFilled(slots, 1, 2)) {
                if (allowAutoRotation) {
                    moveToPosition(0);
                    telemetry.addLine("Slots 2 & 3 full; rotating to position 1");
                } else {
                    telemetry.addLine("Slots 2 & 3 full; waiting to rotate to position 1");
                }
            }
        }

        telemetry.addData("Spindexer Facing", spindexerController.getIndex() + 1);
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

    private void moveToPosition(int index) {
        index = Math.max(0, Math.min(index, 3 - 1));
        spindexerController.setPosition(index);
        long now = System.nanoTime();
        sensorReadAllowedAtNanos = now + SENSOR_SETTLE_TIME_NANOS;
        autoRotateAllowedAtNanos = now + SENSOR_SETTLE_TIME_NANOS;
    }
}
