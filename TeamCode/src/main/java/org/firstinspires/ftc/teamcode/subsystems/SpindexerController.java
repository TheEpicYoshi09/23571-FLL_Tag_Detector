package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class SpindexerController {
    private final RobotHardware robot;
    private final Telemetry telemetry;

    private final double[] spindexerPositions = new double[]{Constants.spindexer1, Constants.spindexer2, Constants.spindexer3};
    private int spindexerIndex = 0;
    private double spindexerPos = Constants.spindexerStart;

    public SpindexerController(RobotHardware robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void init() {
        setPosition(0);
    }

    public void update() {
        telemetry.addData("SPINDEXER POSITION", spindexerPos);
        telemetry.addData("SPINDEXER INDEX", spindexerIndex);
    }

    public void setPosition(int index) {
        int newIndex = index % 3;
        if (spindexerIndex == newIndex) return; // this probably does nothing but im hoping it'll stop it from trying to set the position to the current position, and draw less power or something????????
        spindexerIndex = newIndex;
        robot.spindexer.setPosition(spindexerPositions[newIndex]);
        spindexerPos = spindexerPositions[newIndex];
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
