package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Represents an FTC servo or continuous rotation (CR) servo with two preset states.
 */
class ServoClass {
    public enum ServoType { STANDARD, CONTINUOUS_ROTATION }

    private ServoType type;
    private String hardwareMapName;
    private double pos1;
    private double pos2;

    // HardwareDevice is the common interface for ALL hardware
    private HardwareDevice device;
    private Servo standardServo;
    private CRServo crServo;

    public ServoClass(ServoType type, String name, double pos1, double pos2) {
        this.type = type;
        this.hardwareMapName = name;
        this.pos1 = pos1;
        this.pos2 = pos2;
    }

    public void init(HardwareMap hwMap) {
        if (type == ServoType.STANDARD) {
            standardServo = hwMap.get(Servo.class, hardwareMapName);
            device = standardServo;
        } else if (type == ServoType.CONTINUOUS_ROTATION) {
            crServo = hwMap.get(CRServo.class, hardwareMapName);
            device = crServo;
        }
    }

    /**
     * Sets the position for a standard servo or the power for a CR servo.
     *
     * @param position The target position (for standard) or power (for CR).
     */
    public void goToPosition(double position) {
        if (device == null) return;

        if (type == ServoType.STANDARD) {
            standardServo.setPosition(position);
        } else {
            // For a CR servo, the "position" parameter is treated as power.
            crServo.setPower(position);
        }
    }

    public void goToPosition1() {
        goToPosition(pos1);
    }

    public void goToPosition2() {
        goToPosition(pos2);
    }

    public double getPos1() { return pos1; }
    public void setPos1(double pos1) { this.pos1 = pos1; }
    public double getPos2() { return pos2; }
    public void setPos2(double pos2) { this.pos2 = pos2; }
    public String getName() { return hardwareMapName; }

    // Returns the base HardwareDevice
    public HardwareDevice getHardwareDevice() { return device; }
}
