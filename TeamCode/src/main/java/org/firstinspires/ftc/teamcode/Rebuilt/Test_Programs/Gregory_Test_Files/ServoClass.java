package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Represents an FTC servo with two preset positions.
 */
class ServoPreset {
    private String hardwareMapName;
    private double pos1;
    private double pos2;
    private Servo servo; // The actual FTC Hardware Object

    public ServoPreset(String name, double pos1, double pos2) {
        this.hardwareMapName = name;
        this.pos1 = pos1;
        this.pos2 = pos2;
    }

    /**
     * Initializes the servo hardware from the hardwareMap.
     */
    public void init(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, hardwareMapName);
    }

    // Command methods
    public void goToPosition1() {
        if (servo != null) servo.setPosition(pos1);
    }

    public void goToPosition2() {
        if (servo != null) servo.setPosition(pos2);
    }

    // Getters and Setters
    public double getPos1() { return pos1; }
    public void setPos1(double pos1) { this.pos1 = pos1; }
    public double getPos2() { return pos2; }
    public void setPos2(double pos2) { this.pos2 = pos2; }
    public String getName() { return hardwareMapName; }
    public Servo getHardware() { return servo; }
}
