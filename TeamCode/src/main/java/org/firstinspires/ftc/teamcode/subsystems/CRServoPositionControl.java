package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class CRServoPositionControl {
    private final CRServo crServo;
    private final AnalogInput encoder; // Analog input for position from 4th wire

    public static double kp = 0.15;
    public static double ki = 0.0;
    public static double kd = 0.02;
    public static double kf = 0.2;

    private double integral = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.crServo = servo;
        this.encoder = encoder;
        timer.reset();
    }

    private double getCurrentPositionVoltage() {
        return encoder.getVoltage(); // Read analog voltage from 4th wire
    }

    private double angleToVoltage(double angleDegrees) {
        angleDegrees = Math.max(0, Math.min(360, angleDegrees)); // Clamp
        return (angleDegrees / 360.0) * 3.2;
    }

    public void moveToAngle(double targetAngleDegrees) {
        double targetVoltage = angleToVoltage(targetAngleDegrees);
        double currentVoltage = getCurrentPositionVoltage();

        double error = targetVoltage - currentVoltage;

        // Shortest path wrap handling, for continuous rotation (optional)
        if (error > 1.65) { error -= 3.3; }
        if (error < -1.65) { error += 3.3; }

        double deltaTime = timer.seconds();
        timer.reset();
        integral += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;

        double output = kp * error + ki * integral + kd * derivative + kf * Math.signum(error);
        output = Math.max(-1.0, Math.min(1.0, output));
        crServo.setPower(output);
        lastError = error;
    }
}
