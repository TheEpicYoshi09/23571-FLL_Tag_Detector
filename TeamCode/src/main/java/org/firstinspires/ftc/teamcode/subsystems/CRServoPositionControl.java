package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
public class CRServoPositionControl {

    private final CRServo crServo;
    private final AnalogInput encoder;

    public static double kp = 0.12;
    public static double ff = 0.05;
    public static double deadband = 2.0;
    public static double minPower = 0.05;

    public static double maxVoltage = 3.2;
    public static double fullRotation = 360.0;

    private double continuousAngle = 0.0;
    private Double lastRawAngle = null;
    private double filteredVoltage = 0.0;

    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.crServo = servo;
        this.encoder = encoder;
    }

    public double getCurrentAngle() {
        return getContinuousAngle();
    }

    private double getFilteredVoltage() {
        filteredVoltage = 0.6 * filteredVoltage + 0.4 * encoder.getVoltage();
        return filteredVoltage;
    }

    private double voltageToAngle(double v) {
        return (v / maxVoltage) * fullRotation;
    }

    private double getContinuousAngle() {
        double rawAngle = voltageToAngle(getFilteredVoltage());

        if (lastRawAngle == null) {
            lastRawAngle = rawAngle;
            continuousAngle = rawAngle;
            return continuousAngle;
        }

        double delta = rawAngle - lastRawAngle;

        // unwrap rollover
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        continuousAngle += delta;
        lastRawAngle = rawAngle;

        return continuousAngle;
    }

    public void moveToAngle(double targetDeg) {
        double current = getContinuousAngle();
        double error = targetDeg - current;

        // unwrap error to shortest path
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // stop if within deadband
        if (Math.abs(error) < deadband) {
            crServo.setPower(0);
            return;
        }

        // proportional + feedforward
        double power = kp * error + ff * Math.signum(error);

        // apply minimum power
        if (Math.abs(power) < minPower) {
            power = minPower * Math.signum(power);
        }

        // clamp to -1,1
        power = Math.max(-1, Math.min(1, power));

        crServo.setPower(power);
    }
}


/*
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class CRServoPositionControl {
    private final CRServo crServo;
    private final AnalogInput encoder;

    private static boolean angleIsLocked = false;
    private static double lockedAngle;
    public static double kp = 0.341;
    public static double ki = 0.0;
    public static double kd = 0.0;
    public static double kf = 0.0167;
    public static double lockedkp = 0.341;
    public static double lockedki = 0.0;
    public static double lockedkd = 0.0;
    public static double lockedkf = 0.0167;
    public static double filterAlpha = 0.80;
    public static double angleDeadband = 1.67;


    // Dynamic speed parameters
    public static double minSpeed = 0.15;           // minimal power to overcome deadband
    public static double maxErrorForScaling = 90.0; // error threshold for full speed

    private double integral = 0.0;
    private double lastError = 0.0;
    private Double filteredVoltage = null;
    public final double MAX_VOLTAGE;
    private ElapsedTime timer = new ElapsedTime();
    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.crServo = servo;
        this.encoder = encoder;
        this.MAX_VOLTAGE = encoder.getMaxVoltage();
        timer.reset();
    }

    private double getFilteredVoltage() {
        if (filteredVoltage == null) {
            filteredVoltage = encoder.getVoltage();  // initialize to first reading
        } else {
            filteredVoltage = (1 - filterAlpha) * filteredVoltage + filterAlpha * encoder.getVoltage();
        }
        return filteredVoltage;
    }

    double getAngle() {
        return (getFilteredVoltage() / MAX_VOLTAGE) * 360.0;
    }

    public void lockAngle(double targetAngleDegrees) {
        angleIsLocked = true;
        lockedAngle = targetAngleDegrees;
    }
    public void unlockAngle() {
        angleIsLocked = false;
    }

    public void moveToAngle(double targetAngleDegrees) {
        double tempkp = kp;
        double tempki = ki;
        double tempkd = kd;
        double tempkf = kf;
        if (angleIsLocked) {
            targetAngleDegrees = lockedAngle;
            tempkp = lockedkp;
            tempki = lockedki;
            tempkd = lockedkd;
            tempkf = lockedkf;
        }

        double currentAngle = getAngle();
        double error = ((targetAngleDegrees - currentAngle + 540) % 360) - 180;

        if (Math.abs(error) < angleDeadband) {
            crServo.setPower(0);
            integral = 0;
            lastError = error;
            return;
        }

        double deltaTime = timer.seconds();
        timer.reset();
        if (deltaTime <= 0.0001) deltaTime = 0.0001;

        integral += error * deltaTime;
        integral = Math.max(-2, Math.min(2, integral));

        double derivative = (error - lastError) / deltaTime;

        double output = tempkp * error + tempki * integral + tempkd * derivative + tempkf * Math.signum(error);

        // Dynamic speed scaling, speeds up if distance is further
        double distanceFactor = Math.min(Math.abs(error) / maxErrorForScaling, 1.0);
        double scaledPower = minSpeed + (1.0 - minSpeed) * distanceFactor;
        output = Math.signum(output) * Math.min(Math.abs(output), scaledPower);

        crServo.setPower(output);
        lastError = error;
    }
}
*/