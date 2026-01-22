package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class SpindexerController {
    private final RobotHardware robot;
    private final ArtifactTracker artifactTracker;
    private final Telemetry telemetry;

    private final double[] spindexerPositions = new double[]{Constants.SPINDEXER_1, Constants.SPINDEXER_2, Constants.SPINDEXER_3};
    private int spindexerIndex = 0;
    private double spindexerPos = Constants.SPINDEXER_1;
    private boolean autoSpinEnabled = false;
    private final Timer autoTimer = new Timer();

    public SpindexerController(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.artifactTracker = new ArtifactTracker(robot, telemetry);
        this.telemetry = telemetry;
    }

    public void init() {
        setPosition(0);
    }

    public void update() {
        artifactTracker.update();

        telemetry.addLine("--- SPINDEXER ---");

        if (autoSpinEnabled) autoUpdate();

        telemetry.addData("SPINDEXER POSITION", spindexerPos);
        telemetry.addData("SPINDEXER INDEX", spindexerIndex);
        telemetry.addLine("-------------------------");
    }

    public void autoUpdate() {
        telemetry.addLine("AUTO SPINDEXER ENABLED!");

        if (autoTimer.getElapsedTimeSeconds() < 0.35)  {
            telemetry.addLine("AUTO ISNT ENAGAGED!");
            return;
        }

        if (isSpindexerFull()) {
            telemetry.addLine("SPINDEXER IS FULL!");
            return;
        }

        int currentIndex = getIndex();
        int lastIndex = getPreviousIndex(currentIndex);
        int nextIndex = getNextIndex(currentIndex);

        boolean lastSlotEmpty = getSlotState(lastIndex) == ArtifactTracker.SlotStatus.VACANT;
        boolean currentSlotFull = getCurrentSlotState() != ArtifactTracker.SlotStatus.VACANT;
        boolean nextSlotFull = getSlotState(nextIndex) != ArtifactTracker.SlotStatus.VACANT;

        telemetry.addData("CHECKING INDEX", nextIndex);

        if (currentSlotFull && nextSlotFull && lastSlotEmpty) {
            setPosition(nextIndex);
            autoTimer.resetTimer();
        }
    }

    public int getPreviousIndex(int index) {
        int previousIndex = index - 1;
        if (previousIndex < 0) previousIndex = 2;
        return previousIndex;
    }

    public int getNextIndex(int index) {
        int nextIndex = index + 1;
        if (nextIndex > 2) nextIndex = 0;
        return nextIndex;
    }

    public ArtifactTracker getArtifactTracker() {
        return artifactTracker;
    }

    public ArtifactTracker.SlotStatus getSlotState(int index) {
        return artifactTracker.getSlotStatus(index);
    }

    public ArtifactTracker.SlotStatus getCurrentSlotState() {
        return artifactTracker.getSlotStatus(getIndex());
    }

    public void toggleAuto() {
        autoSpinEnabled = !autoSpinEnabled;
    }

    public void disableAuto() {
        autoSpinEnabled = false;
    }

    public void enableAuto() {
        autoSpinEnabled = true;
    }

    public boolean isEnabled() {
        return autoSpinEnabled;
    }

    public boolean isSpindexerFull() {
        return getSlotState(0) != ArtifactTracker.SlotStatus.VACANT
                && getSlotState(1) != ArtifactTracker.SlotStatus.VACANT
                && getSlotState(2) != ArtifactTracker.SlotStatus.VACANT;
    }

    public boolean isSpindexerEmpty() {
        return getSlotState(0) == ArtifactTracker.SlotStatus.VACANT
                && getSlotState(1) == ArtifactTracker.SlotStatus.VACANT
                && getSlotState(2) == ArtifactTracker.SlotStatus.VACANT;
    }

    public void setPosition(int index) {
        spindexerIndex = Math.floorMod(index, 3);
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
