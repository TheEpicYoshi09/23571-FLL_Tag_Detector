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

//    private long sensorReadAllowedAtNanos = 0L;
//    private long autoRotateAllowedAtNanos = 0L;
//    private static final long SENSOR_SETTLE_TIME_NANOS = 500_000_000L; // 0.5 seconds
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

        if (autoSpinEnabled) autoUpdate();

        telemetry.addData("SPINDEXER POSITION", spindexerPos);
        telemetry.addData("SPINDEXER INDEX", spindexerIndex);
        telemetry.addLine("-------------------------");
    }

    public void autoUpdate() {
        telemetry.addLine("AUTO SPINDEXER ENABLED!");

        // help me
        int nextIndex = getIndex() + 1;
        int lastSlot = 0;
        if (nextIndex > 2) {
            nextIndex = 0;
            lastSlot = 1;
        } else if (nextIndex == 2) {
            nextIndex = 1;
            lastSlot = 2;
        } else {
            nextIndex = 2;
        }

        telemetry.addData("CHECKING INDEX", nextIndex);

        if (artifactTracker.getSlotStatus(getIndex()) != ArtifactTracker.SlotStatus.VACANT
                && artifactTracker.getSlotStatus(nextIndex) != ArtifactTracker.SlotStatus.VACANT
            && artifactTracker.getSlotStatus(lastSlot) == ArtifactTracker.SlotStatus.VACANT)
        {
            setPosition(nextIndex);
        }
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
        spindexerPos -= delta;
        spindexerPos = Math.max(0.0, Math.min(1.0, spindexerPos));
        robot.spindexer.setPosition(spindexerPos);
    }
}
