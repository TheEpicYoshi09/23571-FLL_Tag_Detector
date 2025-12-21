package org.firstinspires.ftc.teamcode.decode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterAngle {

    private Servo angleServo;

    // Change this if your servo is reversed or needs tuning
    private double currentPos = 0.5;

    public void init(HardwareMap hw) {
        angleServo = hw.get(Servo.class, "shooterAngle");
        angleServo.setPosition(currentPos);
    }

    // ---- Set shooter angle ----
    public void setAngle(double pos) {
        currentPos = pos;
        angleServo.setPosition(pos);
    }

    // ---- Adjust angle up/down ----
    public void nudge(double amount) {
        currentPos += amount;
        currentPos = Math.max(0.0, Math.min(1.0, currentPos));
        angleServo.setPosition(currentPos);
    }

    // ---- Read current angle ----
    public double getPosition() {
        return currentPos;
    }
}
